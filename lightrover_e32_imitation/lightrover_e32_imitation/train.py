"""記録したデモから方策 (MLP) を学習するスクリプト。

ROS には依存しません (numpy + PyTorch のみ)。ROS 2 を動かしているPCでも、
別のGPUマシンや venv でも、データフォルダさえあれば学習できます。

例 (venv の Python で直接実行。README「準備」参照):
  ~/il_venv/bin/python train.py --data ~/il_data --out ~/il_models/model.npz --mirror
  # システムの Python に PyTorch が入っている環境なら ros2 run でも可:
  ros2 run lightrover_e32_imitation il_train --data ~/il_data --mirror
"""

import argparse
import glob
import os
import sys

import numpy as np

try:
    from lightrover_e32_imitation.features import batch_scan_to_sectors, mirror
    from lightrover_e32_imitation.policy_model import NumpyPolicy, save_npz
except ImportError:  # パッケージ外から直接実行したとき
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from features import batch_scan_to_sectors, mirror
    from policy_model import NumpyPolicy, save_npz


def load_files(paths, args):
    """各ファイルを特徴量に変換し、ファイルごとに末尾 val_frac を検証用に分ける。

    ランダムに分けると、隣り合う (ほぼ同じ) フレームが学習・検証の両方に入り
    検証スコアが甘くなるので、時間で区切っています。
    """
    tr, va = [], []
    for path in paths:
        d = np.load(path)
        if len(d['actions']) == 0:
            continue
        f = batch_scan_to_sectors(d['ranges'], float(d['angle_min']), float(d['angle_increment']),
                                  n_sectors=args.sectors, max_range=args.max_range,
                                  laser_yaw=float(d['laser_yaw']))
        a = d['actions'].astype(np.float32)
        w = np.where(d['intervention'], args.intervention_weight, 1.0).astype(np.float32)
        n = len(a)
        k = int(n * (1.0 - args.val_frac))
        tr.append((f[:k], a[:k], w[:k]))
        va.append((f[k:], a[k:], w[k:]))
        print(f'  {os.path.basename(path)}: {n} サンプル '
              f'(割り込み {int(d["intervention"].sum())})')

    def cat(parts):
        return tuple(np.concatenate([p[i] for p in parts]) for i in range(3))
    return cat(tr), cat(va)


def normalize_actions(a, args):
    lim = np.array([args.max_linear, args.max_angular], dtype=np.float32)
    # tanh の出力範囲 [-1, 1] に合わせる。ぴったり ±1 は tanh が出せないので少し内側に。
    return np.clip(a / lim, -0.98, 0.98)


def main(argv=None):
    ap = argparse.ArgumentParser(description='ライトローバーE32 模倣学習 (LiDAR -> 速度指令)')
    ap.add_argument('--data', nargs='+', default=['~/il_data'],
                    help='データフォルダ または .npz ファイル (複数可)')
    ap.add_argument('--out', default='~/il_models/model.npz')
    ap.add_argument('--sectors', type=int, default=36, help='360°を何分割するか')
    ap.add_argument('--max-range', type=float, default=1.0,
                    help='この距離[m]より遠い物は「遠い」として同一視 (卓上なら 0.5〜1.0)')
    ap.add_argument('--hidden', type=int, nargs='+', default=[64, 64])
    ap.add_argument('--epochs', type=int, default=300)
    ap.add_argument('--batch', type=int, default=128)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val-frac', type=float, default=0.15)
    ap.add_argument('--mirror', action='store_true', help='左右反転でデータを2倍にする')
    ap.add_argument('--intervention-weight', type=float, default=2.0,
                    help='AI走行中に人が修正したサンプルの重み')
    ap.add_argument('--max-linear', type=float, default=0.10, help='twist_limiter と同じ値に')
    ap.add_argument('--max-angular', type=float, default=1.0, help='twist_limiter と同じ値に')
    ap.add_argument('--plot', default=None, help='検証データの予測グラフを保存するPNGパス')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args(argv)

    try:
        import torch
        import torch.nn as nn
    except ImportError:
        sys.exit('PyTorch が見つかりません。 pip install torch (CPU版で十分) を実行してください。')

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    paths = []
    for p in args.data:
        p = os.path.expanduser(p)
        paths += sorted(glob.glob(os.path.join(p, '*.npz'))) if os.path.isdir(p) else [p]
    paths = [p for p in paths if not p.endswith('.tmp.npz')]
    if not paths:
        sys.exit('データが見つかりません: ' + ' '.join(args.data))

    print(f'データ読み込み ({len(paths)} ファイル)')
    (xtr, atr, wtr), (xva, ava, wva) = load_files(paths, args)
    if args.mirror:
        xm, am = mirror(xtr, atr)
        xtr, atr, wtr = np.concatenate([xtr, xm]), np.concatenate([atr, am]), np.concatenate([wtr, wtr])
    if len(xva) == 0:
        print('検証データが0件なので、学習データで評価します (データが少なすぎます)')
        xva, ava, wva = xtr, atr, wtr
    ytr, yva = normalize_actions(atr, args), normalize_actions(ava, args)
    print(f'学習 {len(xtr)} / 検証 {len(xva)} サンプル  (入力 {args.sectors} 次元)')

    # --- モデル ------------------------------------------------------------
    dims = [args.sectors] + args.hidden + [2]
    linears = [nn.Linear(i, o) for i, o in zip(dims[:-1], dims[1:])]
    layers = []
    for i, lin in enumerate(linears):
        layers.append(lin)
        layers.append(nn.ReLU() if i < len(linears) - 1 else nn.Tanh())
    model = nn.Sequential(*layers)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-5)

    Xtr, Ytr, Wtr = map(torch.from_numpy, (xtr, ytr, wtr))
    Xva, Yva, Wva = map(torch.from_numpy, (xva, yva, wva))

    def wmse(pred, y, w):
        return ((pred - y) ** 2).mean(dim=1).mul(w).sum() / w.sum()

    best, best_state, patience = float('inf'), None, 0
    for ep in range(1, args.epochs + 1):
        model.train()
        perm = torch.randperm(len(Xtr))
        for s in range(0, len(Xtr), args.batch):
            idx = perm[s:s + args.batch]
            loss = wmse(model(Xtr[idx]), Ytr[idx], Wtr[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            tr_loss = wmse(model(Xtr), Ytr, Wtr).item()
            va_loss = wmse(model(Xva), Yva, Wva).item() if len(Xva) else tr_loss
        if va_loss < best - 1e-5:
            best, patience = va_loss, 0
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
        else:
            patience += 1
        if ep % 25 == 0 or ep == 1:
            print(f'  epoch {ep:4d}  train {tr_loss:.4f}  val {va_loss:.4f}')
        if patience >= 60:
            print(f'  epoch {ep}: 検証誤差が改善しなくなったので終了')
            break
    model.load_state_dict(best_state)

    # --- 保存 (numpy 形式) ----------------------------------------------------
    out = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    save_npz(out, linears, dict(
        n_sectors=args.sectors, max_range=args.max_range, laser_yaw=float(np.load(paths[0])['laser_yaw']),
        max_linear=args.max_linear, max_angular=args.max_angular))

    # --- 評価 ----------------------------------------------------------------
    pol = NumpyPolicy(out)
    with torch.no_grad():
        torch_pred = model(Xva).numpy() * np.array([args.max_linear, args.max_angular])
    pred = pol(xva)
    assert np.allclose(pred, torch_pred, atol=1e-4), 'numpy 推論と PyTorch の結果が一致しません'

    target = np.clip(ava, [-args.max_linear, -args.max_angular], [args.max_linear, args.max_angular])
    mae = np.abs(pred - target).mean(axis=0)
    base = np.abs(target.mean(axis=0) - target).mean(axis=0)
    turning = np.abs(target[:, 1]) > 0.2
    sign_acc = (np.sign(pred[turning, 1]) == np.sign(target[turning, 1])).mean() if turning.any() else float('nan')
    print()
    print(f'保存しました: {out}')
    print(f'検証データでの平均誤差   直進 {mae[0]:.3f} m/s   旋回 {mae[1]:.3f} rad/s')
    print(f'(参考: 常に平均値を出すだけの場合  直進 {base[0]:.3f} m/s   旋回 {base[1]:.3f} rad/s)')
    print(f'曲がる場面で左右を当てた割合: {sign_acc * 100:.0f}%')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
        for i, (name, unit) in enumerate([('linear.x', 'm/s'), ('angular.z', 'rad/s')]):
            ax[i].plot(target[:, i], label='human', lw=1.5)
            ax[i].plot(pred[:, i], label='policy', lw=1.2)
            ax[i].set_ylabel(f'{name} [{unit}]')
            ax[i].legend(loc='upper right')
        ax[1].set_xlabel('validation sample')
        fig.tight_layout()
        fig.savefig(os.path.expanduser(args.plot), dpi=120)
        print(f'グラフ: {args.plot}')


if __name__ == '__main__':
    main()
