"""LiDARスキャン -> 学習用特徴量 への変換。

ROSに依存しない (numpyのみ) ので、記録ノード・学習スクリプト・推論ノードの
すべてで同じ関数を使います。前処理が学習時と推論時で食い違うのは
模倣学習でよくあるバグなので、ここ1か所にまとめています。

特徴量の作り方:
  1. 各点の角度を「ロボット正面 = 0 rad, 左回り正」の座標に直す
     (ライトローバーE32の laser_frame は base_link に対して -90° 回っている)
  2. 360° を n_sectors 個の扇形に分け、扇形ごとの最小距離をとる
     sector 0 = 正面, sector n/4 = 左, sector n/2 = 後ろ, sector 3n/4 = 右
  3. max_range で頭打ちにして 0〜1 に正規化
"""

import numpy as np

# URDF (lightrover_e32_description) の base_to_laser joint: rpy="0 0 -1.5708"
DEFAULT_LASER_YAW = -np.pi / 2.0


def scan_to_sectors(ranges, angle_min, angle_increment,
                    n_sectors=36, max_range=1.0, laser_yaw=DEFAULT_LASER_YAW):
    """1スキャン分の距離配列を、扇形ごとの最小距離 (0〜1) に変換する。

    ranges: (P,) 距離[m]。inf / nan / 0 は「計測なし」として扱う。
    戻り値: (n_sectors,) float32。計測のない扇形は 1.0 (= max_range 以上)。
    """
    r = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + np.arange(r.shape[0]) * angle_increment + laser_yaw
    width = 2.0 * np.pi / n_sectors
    # 扇形の中心が 0, width, 2*width ... になるように round で割り当てる
    idx = np.round(angles / width).astype(np.int64) % n_sectors

    valid = np.isfinite(r) & (r > 0.0)
    out = np.full(n_sectors, max_range, dtype=np.float64)
    if np.any(valid):
        np.minimum.at(out, idx[valid], r[valid])
    out = np.clip(out, 0.0, max_range) / max_range
    return out.astype(np.float32)


def batch_scan_to_sectors(ranges_2d, angle_min, angle_increment, **kw):
    """(N, P) のスキャン配列をまとめて変換する。"""
    return np.stack([scan_to_sectors(r, angle_min, angle_increment, **kw)
                     for r in ranges_2d]).astype(np.float32)


def mirror(features, actions):
    """左右反転したデータを作る (データ拡張)。

    特徴量: sector i <-> sector -i を入れ替え
    行動  : 角速度 angular.z の符号を反転 (直進速度はそのまま)
    「右の壁に沿って走る」デモから「左の壁に沿って走る」デモが手に入る。
    """
    features = np.asarray(features)
    n = features.shape[-1]
    perm = (-np.arange(n)) % n
    f2 = features[..., perm]
    a2 = np.array(actions, dtype=np.float32, copy=True)
    a2[..., 1] *= -1.0
    return f2, a2


def sector_min(features, center_deg, half_width_deg, n_sectors=None, max_range=1.0):
    """指定方向 (度, 正面0・左正) 付近の最小距離[m]を返す。安全停止やログ表示用。"""
    f = np.asarray(features)
    n = n_sectors or f.shape[-1]
    width = 360.0 / n
    k = int(round(half_width_deg / width))
    c = int(round(center_deg / width))
    idx = [(c + j) % n for j in range(-k, k + 1)]
    return float(np.min(f[idx]) * max_range)
