import os
import subprocess

import numpy as np
import pytest

from ctrlbench import bench_py as B
from ctrlbench import common as C

BUILD = os.environ.get("CTRLBENCH_BUILD", "build")
needs_model = pytest.mark.skipif(not os.path.isfile(C.UR3_XML), reason="UR3 MJCF not found, set CTRLBENCH_UR3_XML")
needs_build = pytest.mark.skipif(not os.path.isfile(f"{BUILD}/ik_dh"), reason="C++ not built (make build)")


def test_stats_percentiles_and_jitter():
    s = C.stats(np.arange(1, 101))
    assert s["n"] == 100 and s["median"] == pytest.approx(50.5)
    assert s["p95"] == pytest.approx(95.05) and s["p99"] == pytest.approx(99.01)
    assert s["jitter"] == pytest.approx(s["p99"] - s["median"])


def test_dh_fk_home_pose_is_a_rotation():
    R, p = C.fk(C.Q_HOME)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12) and np.linalg.det(R) == pytest.approx(1.0)
    assert np.linalg.norm(p) < 0.6          # inside the UR3's reach


def test_ik_targets_are_reproducible():
    assert np.array_equal(C.make_ik_targets(10, seed=3), C.make_ik_targets(10, seed=3))
    assert not np.array_equal(C.make_ik_targets(10, seed=3), C.make_ik_targets(10, seed=4))


def test_numpy_ik_reaches_the_target_pose():
    t = C.make_ik_targets(15)
    for row in t:
        pt, Rt = row[:3], row[3:12].reshape(3, 3)
        q, it = B.solve_dh(C.Q_HOME, pt, Rt)
        R, p = C.fk(q)
        if it < C.IK_MAX_ITER:
            assert np.linalg.norm(p - pt) < 1e-3 and np.linalg.norm(C.rot_err(R, Rt)) < 1e-3


@needs_model
def test_mujoco_and_dh_ik_agree_on_iteration_counts():
    t = C.make_mj_targets(10)
    r = B.run_ik_mj(t)
    d = B.run_ik_dh(C.make_ik_targets(10))
    assert np.array_equal(r[:, 1], d[:, 1])          # same kinematics, same solver, same path


@needs_model
def test_pinocchio_path_matches_the_precomputed_law():
    s = C.make_states(25)
    a, b = B.run_imp_law(s), B.run_imp_pin(s)
    assert np.allclose(a[:, 2], b[:, 2], atol=1e-8)


@needs_model
@needs_build
def test_cpp_ik_matches_python(tmp_path):
    t = C.make_ik_targets(20)
    inp, out = tmp_path / "t.csv", tmp_path / "o.csv"
    C.write_csv(inp, t)
    subprocess.run([f"{BUILD}/ik_dh", str(inp), str(out)], check=True)
    c = np.loadtxt(out, delimiter=",", skiprows=1)
    p = B.run_ik_dh(t)
    assert np.array_equal(c[:, 1], p[:, 1]) and np.allclose(c[:, 2], p[:, 2], atol=1e-9)


@needs_model
@needs_build
def test_cpp_impedance_matches_python(tmp_path):
    s = C.make_states(20)
    inp = tmp_path / "s.csv"
    C.write_csv(inp, s)
    for mode, fn in (("law", B.run_imp_law), ("pin", B.run_imp_pin)):
        out = tmp_path / f"{mode}.csv"
        subprocess.run([f"{BUILD}/imp", mode, C.UR3_XML, str(inp), str(out)], check=True)
        c = np.loadtxt(out, delimiter=",", skiprows=1)
        assert np.allclose(c[:, 2], fn(s)[:, 2], atol=1e-8)
