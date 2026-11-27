"""シミュレータ用のコース定義と、LiDAR のレイキャスト。

ROS に依存しない (numpy と PyYAML のみ) ので、シミュレータノードとテストの両方で使います。

コースは YAML で定義します (share/lightrover_e32_imitation/courses/*.yaml)。

  name: square
  description: 四角いドーナツ (通路幅 0.4m)
  polygons:              # 閉じた多角形の壁 (外周や中島)。頂点を順に並べる
    - [[-0.65, -0.65], [0.65, -0.65], [0.65, 0.65], [-0.65, 0.65]]
  boxes:                 # 箱の障害物 [xmin, ymin, xmax, ymax]
    - [0.5, -0.05, 0.65, 0.05]
  segments:              # 単独の壁 [x1, y1, x2, y2]
    - [0.0, 0.0, 0.3, 0.0]
  start: [0.45, 0.0, 90]  # 開始位置 x[m], y[m], 向き[度] (x軸正方向が0度、反時計回りが正)
  lap_center: [0.0, 0.0]  # 周回数を数える基準点。中島の内側の点を指定する

単位はすべてメートル。自作のコースも同じ形式で書けば読み込めます。
"""

import os
from dataclasses import dataclass

import numpy as np


@dataclass
class Course:
    name: str
    walls: np.ndarray          # (W, 2, 2) 壁の線分
    start: tuple               # (x, y, yaw[rad])
    lap_center: tuple          # (x, y)
    description: str = ''


def _polygon_segments(points):
    pts = [tuple(map(float, p)) for p in points]
    return [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]


def course_from_dict(d, name='custom'):
    segs = []
    for poly in d.get('polygons', []) or []:
        segs += _polygon_segments(poly)
    for b in d.get('boxes', []) or []:
        x0, y0, x1, y1 = map(float, b)
        segs += _polygon_segments([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    for s in d.get('segments', []) or []:
        x0, y0, x1, y1 = map(float, s)
        segs.append(((x0, y0), (x1, y1)))
    if not segs:
        raise ValueError(f'コース {name}: 壁が1つも定義されていません')
    sx, sy, syaw = (list(map(float, d.get('start', [0.0, 0.0, 0.0]))) + [0.0])[:3]
    cx, cy = map(float, d.get('lap_center', [0.0, 0.0]))
    return Course(name=str(d.get('name', name)), walls=np.asarray(segs, dtype=np.float64),
                  start=(sx, sy, np.deg2rad(syaw)), lap_center=(cx, cy),
                  description=str(d.get('description', '')))


def load_course(name_or_path, search_dirs=()):
    """コース名 (例: 'square') または YAML ファイルのパスからコースを読み込む。"""
    import yaml
    path = os.path.expanduser(name_or_path)
    if not os.path.isfile(path):
        for d in search_dirs:
            cand = os.path.join(d, f'{name_or_path}.yaml')
            if os.path.isfile(cand):
                path = cand
                break
        else:
            avail = sorted({os.path.splitext(f)[0] for d in search_dirs if os.path.isdir(d)
                            for f in os.listdir(d) if f.endswith('.yaml')})
            raise FileNotFoundError(
                f'コース "{name_or_path}" が見つかりません。利用できるコース: {", ".join(avail)}')
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    return course_from_dict(d, name=os.path.splitext(os.path.basename(path))[0])


def raycast(walls, x, y, yaw, n_points=360, laser_offset_x=-0.030, laser_yaw=-np.pi / 2):
    """E32 の /scan と同じ並びで、壁までの真の距離[m]を返す (当たらない方向は inf)。

    並び: laser_frame 基準で angle_min=0 から反時計回りに 360/n_points 度刻み。
    laser_frame は base_link から x 方向に laser_offset_x、向きは laser_yaw 回転 (E32 の URDF)。
    """
    lx = x + laser_offset_x * np.cos(yaw)
    ly = y + laser_offset_x * np.sin(yaw)
    ang = yaw + laser_yaw + np.arange(n_points) * (2 * np.pi / n_points)
    d = np.stack([np.cos(ang), np.sin(ang)], 1)                       # (P, 2)
    p, q = walls[:, 0], walls[:, 1]                                   # (W, 2)
    s = q - p
    o = p - np.array([lx, ly])
    den = d[:, None, 0] * s[None, :, 1] - d[:, None, 1] * s[None, :, 0]   # (P, W)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (o[None, :, 0] * s[None, :, 1] - o[None, :, 1] * s[None, :, 0]) / den
        u = (o[None, :, 0] * d[:, None, 1] - o[None, :, 1] * d[:, None, 0]) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    return np.where(hit, t, np.inf).min(axis=1)


def clearance(walls, x, y):
    """点 (x, y) から一番近い壁までの距離[m]。"""
    p, q = walls[:, 0], walls[:, 1]
    s = q - p
    pt = np.array([x, y])
    L2 = np.maximum((s ** 2).sum(1), 1e-12)
    t = np.clip(((pt - p) * s).sum(1) / L2, 0.0, 1.0)
    proj = p + t[:, None] * s
    return float(np.sqrt(((proj - pt) ** 2).sum(1)).min())


def winding_delta(center, x0, y0, x1, y1):
    """基準点まわりの角度の変化[rad]。積算して 2π で割ると周回数になる (反時計回りが正)。"""
    cx, cy = center
    a0 = np.arctan2(y0 - cy, x0 - cx)
    a1 = np.arctan2(y1 - cy, x1 - cx)
    return float((a1 - a0 + np.pi) % (2 * np.pi) - np.pi)
