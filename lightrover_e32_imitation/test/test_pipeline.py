import numpy as np
import pytest

from lightrover_e32_imitation.features import mirror, scan_to_sectors, sector_min
from lightrover_e32_imitation import sim2d


def test_orientation_matches_e32_urdf():
    # コース右辺の中央で北向き: 右に外壁(0.2m)、左に内壁(0.2m)、前方は 0.65m 先の外壁
    r = sim2d.raycast(0.45, 0.0, np.pi / 2)
    f = scan_to_sectors(r, 0.0, 2 * np.pi / 360, n_sectors=36, max_range=1.0)
    assert sector_min(f, 90, 5) == pytest.approx(0.20, abs=0.01)
    assert sector_min(f, -90, 5) == pytest.approx(0.20, abs=0.01)
    assert sector_min(f, 0, 5) == pytest.approx(0.68, abs=0.01)


def test_mirror_swaps_left_right_and_negates_turn():
    r = sim2d.raycast(0.50, 0.0, np.pi / 2)          # 右の壁に近い
    f = scan_to_sectors(r, 0.0, 2 * np.pi / 360)
    fm, am = mirror(f[None], np.array([[0.1, 0.5]], np.float32))
    assert sector_min(fm[0], 90, 5) == pytest.approx(sector_min(f, -90, 5))
    assert am[0, 1] == pytest.approx(-0.5)
    assert am[0, 0] == pytest.approx(0.1)


def test_invalid_ranges_become_far():
    r = np.full(360, np.inf, np.float32)
    f = scan_to_sectors(r, 0.0, 2 * np.pi / 360)
    assert np.all(f == 1.0)


def test_train_and_drive_in_sim(tmp_path):
    pytest.importorskip('torch')
    from lightrover_e32_imitation import train
    data = tmp_path / 'data'
    sim2d.main(['record', '--out', str(data / 'd.npz'), '--episodes', '4'])
    model = tmp_path / 'm.npz'
    train.main(['--data', str(data), '--out', str(model), '--mirror', '--epochs', '150'])
    laps, crashed = sim2d.main(['eval', '--model', str(model), '--steps', '800'])
    assert not crashed and laps > 1.5
