"""Python implementations of the benchmarks. Each writes one row per call: time_ns, iterations (IK) or 0, checksum.

  python -m ctrlbench.bench_py ik_dh  data/ik_targets.csv  out.csv
  python -m ctrlbench.bench_py ik_mj  data/mj_targets.csv  out.csv
  python -m ctrlbench.bench_py imp_law data/states.csv out.csv
  python -m ctrlbench.bench_py imp_pin data/states.csv out.csv

Each benchmark does one untimed warm-up pass over the inputs, then one timed pass.
"""
import sys
import time

import numpy as np

from ctrlbench import common as C

perf = time.perf_counter_ns


# ---------- IK on the UR3 DH model ----------
def solve_dh(q0, pt, Rt):
    q = q0.copy()
    for it in range(C.IK_MAX_ITER):
        T = C.fk_frames(q)
        e = np.concatenate([pt - T[6][:3, 3], C.rot_err(T[6][:3, :3], Rt)])
        if np.linalg.norm(e) < C.IK_TOL:
            return q, it
        q += C.dls_step(C.jac_dh(T), e)
    return q, C.IK_MAX_ITER


def run_ik_dh(targets):
    def one(row):
        q, it = solve_dh(C.Q_HOME, row[:3], row[3:12].reshape(3, 3))
        return it, float(q.sum())
    return time_calls(targets, one)


# ---------- IK through MuJoCo's Python bindings ----------
def run_ik_mj(targets):
    import mujoco as mj
    m = mj.MjModel.from_xml_path(C.UR3_XML)
    d = mj.MjData(m)
    sid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "attachment_site")
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    eye = np.eye(6)
    lam2 = C.IK_LAMBDA ** 2

    def one(row):
        pt, Rt = row[:3], row[3:12].reshape(3, 3)
        d.qpos[:] = C.Q_HOME
        for it in range(C.IK_MAX_ITER):
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            e = np.concatenate([pt - d.site_xpos[sid], C.rot_err(d.site_xmat[sid].reshape(3, 3), Rt)])
            if np.linalg.norm(e) < C.IK_TOL:
                return it, float(d.qpos.sum())
            mj.mj_jacSite(m, d, jp, jr, sid)
            J = np.vstack([jp, jr])
            d.qpos[:] += J.T @ np.linalg.solve(J @ J.T + lam2 * eye, e)
        return C.IK_MAX_ITER, float(d.qpos.sum())
    return time_calls(targets, one)


# ---------- impedance law ----------
def run_imp_law(states):
    """tau = J^T (K e - D (J v)) + g with J, g, e precomputed: the pure linear-algebra cost."""
    K, D = C.K_GAIN, C.D_GAIN

    def one(row):
        v, J, g, e = row[6:12], row[12:48].reshape(6, 6), row[48:54], row[54:60]
        tau = J.T @ (K * e - D * (J @ v)) + g
        return 0, float(tau.sum())
    return time_calls(states, one)


def run_imp_pin(states):
    """Full Pinocchio path: FK, frame Jacobian, gravity, rotation error, then the law."""
    import pinocchio as pin
    m = pin.buildModelFromMJCF(C.UR3_XML)
    d = m.createData()
    fid = m.getFrameId("attachment_site")
    K, D = C.K_GAIN, C.D_GAIN

    def one(row):
        q, v = row[:6], row[6:12]
        xd, Rd = row[60:63], row[63:72].reshape(3, 3)
        pin.framesForwardKinematics(m, d, q)
        oMf = d.oMf[fid]
        e = np.concatenate([xd - oMf.translation, pin.log3(Rd @ oMf.rotation.T)])
        J = pin.computeFrameJacobian(m, d, q, fid, pin.LOCAL_WORLD_ALIGNED)
        g = pin.computeGeneralizedGravity(m, d, q)
        tau = J.T @ (K * e - D * (J @ v)) + g
        return 0, float(tau.sum())
    return time_calls(states, one)


def time_calls(rows, fn):
    for r in rows:                       # warm-up pass, untimed
        fn(r)
    out = np.empty((len(rows), 3))
    for i, r in enumerate(rows):
        t0 = perf()
        it, chk = fn(r)
        out[i] = (perf() - t0, it, chk)
    return out


def timer_overhead(n=200000):
    t = np.empty(n)
    for i in range(n):
        a = perf()
        t[i] = perf() - a
    return float(np.median(t))


BENCH = dict(ik_dh=run_ik_dh, ik_mj=run_ik_mj, imp_law=run_imp_law, imp_pin=run_imp_pin)


def main(argv):
    name, inp, out = argv
    rows = np.loadtxt(inp, delimiter=",")
    res = BENCH[name](rows)
    np.savetxt(out, res, delimiter=",", fmt="%.17g", header="time_ns,iters,checksum", comments="")


if __name__ == "__main__":
    main(sys.argv[1:])
