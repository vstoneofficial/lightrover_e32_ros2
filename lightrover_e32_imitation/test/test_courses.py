import glob
import os

import numpy as np
import pytest

yaml = pytest.importorskip('yaml')

from lightrover_e32_imitation import courses, sim2d  # noqa: E402

COURSE_DIR = os.path.join(os.path.dirname(__file__), '..', 'courses')
COURSE_FILES = sorted(glob.glob(os.path.join(COURSE_DIR, '*.yaml')))
ROBOT_R = 0.07


def test_course_files_exist():
    names = {os.path.splitext(os.path.basename(p))[0] for p in COURSE_FILES}
    assert {'square', 'square_wide', 'l_shape', 'obstacles'} <= names


@pytest.mark.parametrize('path', COURSE_FILES, ids=os.path.basename)
def test_course_is_drivable(path):
    c = courses.load_course(path)
    x, y, yaw = c.start
    # スタート位置が壁にめり込んでいない
    assert courses.clearance(c.walls, x, y) > ROBOT_R + 0.02
    # スタート位置から LiDAR で全方向に壁が見える (コースが閉じている)
    r = courses.raycast(c.walls, x, y, yaw)
    assert np.all(np.isfinite(r))
    # 周回の基準点は中島の中、スタートは走路上にある。
    # 半直線が壁を横切る回数の偶奇で判定 (走路上なら外周だけを横切るので奇数)
    assert crossings(c.walls, x, y) % 2 == 1
    n = crossings(c.walls, *c.lap_center)
    assert n % 2 == 0 and n >= 2   # 中島と外周を横切る (コースの外ではない)


def crossings(walls, x, y, angle=0.1234567):
    d = np.array([np.cos(angle), np.sin(angle)])
    n = 0
    for p, q in walls:
        s = q - p
        den = d[0] * s[1] - d[1] * s[0]
        if abs(den) < 1e-12:
            continue
        o = p - np.array([x, y])
        t = (o[0] * s[1] - o[1] * s[0]) / den
        u = (o[0] * d[1] - o[1] * d[0]) / den
        n += int(t > 0 and 0 <= u < 1)
    return n


def test_square_course_matches_offline_sim():
    """YAML の square コースと、オフライン用 sim2d.py のコースは同じ形。"""
    c = courses.load_course('square', [COURSE_DIR])
    for (x, y, yaw) in [(0.45, 0.0, np.pi / 2), (0.0, 0.45, np.pi), (-0.4, -0.5, 0.3)]:
        a = courses.raycast(c.walls, x, y, yaw)
        b = sim2d.raycast(x, y, yaw)
        ok = np.isfinite(b)
        np.testing.assert_allclose(a[ok], b[ok], atol=1e-5)
        assert courses.clearance(c.walls, x, y) == pytest.approx(sim2d.clearance(x, y), abs=1e-9)


def test_lap_counting_on_l_shape():
    """L字コースの中央線を1周すると、周回数がちょうど1になる。"""
    c = courses.load_course('l_shape', [COURSE_DIR])
    corners = [(0.0, -0.6), (0.6, -0.6), (0.6, -0.1), (-0.1, -0.1), (-0.1, 0.6),
               (-0.6, 0.6), (-0.6, -0.6), (0.0, -0.6)]
    pts = []
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        for t in np.linspace(0, 1, 50, endpoint=False):
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    pts.append(corners[-1])
    total = sum(courses.winding_delta(c.lap_center, *p, *q) for p, q in zip(pts[:-1], pts[1:]))
    assert total / (2 * np.pi) == pytest.approx(1.0, abs=1e-6)
    # 中央線上はどこも壁から 0.2m 離れている (通路幅 0.4m)
    assert min(courses.clearance(c.walls, x, y) for x, y in pts) == pytest.approx(0.2, abs=1e-6)


def test_unknown_course_lists_available():
    with pytest.raises(FileNotFoundError, match='square'):
        courses.load_course('no_such_course', [COURSE_DIR])
