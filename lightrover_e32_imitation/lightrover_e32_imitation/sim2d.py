"""実機なしで学習パイプラインを試すための、ごく簡単な2Dシミュレータ。

四角いドーナツ形のコース (外側 1.3m 四方 / 内側 0.5m 四方、通路幅 0.4m) を、
「お手本役のコントローラ」が通路の中央を反時計回りに走り、
E32と同じ形式 (360点, angle_min=0, laser_frame は -90°回転) でスキャンを記録します。

  # デモデータを作る (recorder と同じ形式の .npz)
  python3 -m lightrover_e32_imitation.sim2d record --out ~/il_data_sim/demo_sim.npz
  # 学習
  python3 -m lightrover_e32_imitation.train --data ~/il_data_sim --out /tmp/sim_model.npz --mirror
  # 学習した方策で閉ループ走行 (周回数と衝突の有無を表示)
  python3 -m lightrover_e32_imitation.sim2d eval --model /tmp/sim_model.npz --plot /tmp/sim_run.png

物理は「指令した速度がそのまま出る」だけの簡略版です。実機との差 (遅延・滑り・
電池電圧) がない分、ここで動いても実機では動かないことがある、という比較にも使えます。
"""

import argparse
import os
import sys

import numpy as np

try:
    from lightrover_e32_imitation.features import DEFAULT_LASER_YAW, scan_to_sectors
    from lightrover_e32_imitation.policy_model import NumpyPolicy
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from features import DEFAULT_LASER_YAW, scan_to_sectors
    from policy_model import NumpyPolicy

OUTER, INNER = 0.65, 0.25          # 半辺長[m]  (通路幅 0.4m)
ROBOT_R = 0.07                     # ロボット半径 (130mm 四方 → 外接円より少し小さめ)
DT = 0.125                         # スキャン周期 (8Hz)
N_POINTS = 360
RANGE_MIN, RANGE_MAX = 0.12, 8.0
LASER_OFFSET_X = -0.030            # base_link から見た LiDAR の位置


def _box(h):
    c = [(-h, -h), (h, -h), (h, h), (-h, h)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


WALLS = np.array(_box(OUTER) + _box(INNER), dtype=np.float64)   # (8, 2, 2)


def raycast(x, y, th, noise=0.005, rng=None):
    """E32 の /scan と同じ並び (laser_frame 基準, angle_min=0, 1°刻み) で距離を返す。"""
    lx = x + LASER_OFFSET_X * np.cos(th)
    ly = y + LASER_OFFSET_X * np.sin(th)
    ang = th + DEFAULT_LASER_YAW + np.arange(N_POINTS) * (2 * np.pi / N_POINTS)
    d = np.stack([np.cos(ang), np.sin(ang)], 1)                  # (P, 2)
    p, q = WALLS[:, 0], WALLS[:, 1]                              # (W, 2)
    s = q - p
    o = p - np.array([lx, ly])                                   # (W, 2)
    den = d[:, None, 0] * s[None, :, 1] - d[:, None, 1] * s[None, :, 0]   # (P, W)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (o[None, :, 0] * s[None, :, 1] - o[None, :, 1] * s[None, :, 0]) / den
        u = (o[None, :, 0] * d[:, None, 1] - o[None, :, 1] * d[:, None, 0]) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    r = np.where(hit, t, np.inf).min(axis=1)
    if rng is not None:
        r = r + rng.normal(0, noise, r.shape)
        r[rng.random(r.shape) < 0.02] = np.inf                   # ときどき計測抜け
    r[(r < RANGE_MIN) | (r > RANGE_MAX)] = np.inf
    return r.astype(np.float32)


def clearance(x, y):
    """ロボット中心から一番近い壁までの距離。"""
    outer = OUTER - max(abs(x), abs(y))
    dx, dy = max(abs(x) - INNER, 0.0), max(abs(y) - INNER, 0.0)
    inner = np.hypot(dx, dy) if (dx > 0 or dy > 0) else -min(INNER - abs(x), INNER - abs(y))
    return min(outer, inner)


def expert(x, y, th, v=0.08, k=6.0):
    """お手本: 通路の中央線 (半辺長 0.45 の正方形) を反時計回りにたどる。"""
    mid = (OUTER + INNER) / 2
    if abs(x) >= abs(y):     # 左右の辺
        heading = np.pi / 2 if x > 0 else -np.pi / 2
        err = abs(x) - mid
    else:                    # 上下の辺
        heading = np.pi if y > 0 else 0.0
        err = abs(y) - mid
    # 中央より外側にいたら内側 (左) へ寄せる
    target = heading + np.clip(k * err, -0.8, 0.8)
    e = (target - th + np.pi) % (2 * np.pi) - np.pi
    return v, float(np.clip(3.0 * e, -1.0, 1.0))


def lap_angle(x0, y0, x1, y1):
    return (np.arctan2(y1, x1) - np.arctan2(y0, x0) + np.pi) % (2 * np.pi) - np.pi


def record(args):
    rng = np.random.default_rng(args.seed)
    ranges, actions = [], []
    for ep in range(args.episodes):
        x, y = 0.45 + rng.uniform(-0.05, 0.05), rng.uniform(-0.2, 0.2)
        th = np.pi / 2 + rng.uniform(-0.3, 0.3)
        for _ in range(args.steps):
            v, w = expert(x, y, th)
            v += rng.normal(0, 0.005)
            w += rng.normal(0, 0.08)                  # 人の操作のゆらぎ
            ranges.append(raycast(x, y, th, rng=rng))
            actions.append([v, w])
            # 時々わざとふらつかせて「ずれた状態」も記録 (戻り方を覚えさせる)
            w_exec = w + (rng.normal(0, 0.6) if args.perturb and rng.random() < 0.1 else 0.0)
            x += v * np.cos(th) * DT
            y += v * np.sin(th) * DT
            th += w_exec * DT
    out = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    n = len(actions)
    np.savez_compressed(out, ranges=np.stack(ranges), actions=np.asarray(actions, np.float32),
                        stamps=np.arange(n) * DT, intervention=np.zeros(n, bool),
                        angle_min=0.0, angle_increment=2 * np.pi / N_POINTS,
                        laser_yaw=DEFAULT_LASER_YAW)
    print(f'{n} サンプル ({n * DT / 60:.1f} 分相当) -> {out}')


def evaluate(args):
    pol = NumpyPolicy(os.path.expanduser(args.model))
    rng = np.random.default_rng(args.seed + 100)
    x, y, th = 0.45, 0.0, np.pi / 2
    total, traj, prev = 0.0, [(x, y)], np.zeros(2)
    crashed = False
    for i in range(args.steps):
        f = scan_to_sectors(raycast(x, y, th, rng=rng), 0.0, 2 * np.pi / N_POINTS,
                            n_sectors=pol.n_sectors, max_range=pol.max_range,
                            laser_yaw=pol.laser_yaw)
        a = 0.4 * prev + 0.6 * pol(f)      # policy_node と同じスムージング
        prev = a
        nx = x + a[0] * np.cos(th) * DT
        ny = y + a[0] * np.sin(th) * DT
        th += a[1] * DT
        total += lap_angle(x, y, nx, ny)
        x, y = nx, ny
        traj.append((x, y))
        if clearance(x, y) < ROBOT_R * 0.8:
            crashed = True
            print(f'衝突: step {i} ({i * DT:.1f}s)  位置 ({x:.2f}, {y:.2f})')
            break
    laps = total / (2 * np.pi)
    print(f'{"衝突あり" if crashed else "衝突なし"}  周回数 {laps:.2f} '
          f'({len(traj) - 1} step = {(len(traj) - 1) * DT:.0f}s)')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(5, 5))
        for (p, q) in WALLS:
            ax.plot([p[0], q[0]], [p[1], q[1]], 'k-', lw=2)
        t = np.array(traj)
        ax.plot(t[:, 0], t[:, 1], lw=1)
        ax.set_aspect('equal')
        ax.set_title(f'policy run: {laps:.2f} laps, {"crash" if crashed else "no crash"}')
        fig.savefig(os.path.expanduser(args.plot), dpi=110)
        print(f'軌跡: {args.plot}')
    return laps, crashed


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--out', default='~/il_data_sim/demo_sim.npz')
    r.add_argument('--episodes', type=int, default=6)
    r.add_argument('--steps', type=int, default=400)
    r.add_argument('--seed', type=int, default=0)
    r.add_argument('--no-perturb', dest='perturb', action='store_false',
                   help='ふらつきなしの「きれいすぎる」デモにする (複合誤差の実験用)')
    e = sub.add_parser('eval')
    e.add_argument('--model', required=True)
    e.add_argument('--steps', type=int, default=1200)
    e.add_argument('--plot', default=None)
    e.add_argument('--seed', type=int, default=0)
    args = ap.parse_args(argv)
    return record(args) if args.cmd == 'record' else evaluate(args)


if __name__ == '__main__':
    main()
