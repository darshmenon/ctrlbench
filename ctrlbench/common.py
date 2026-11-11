"""Shared helpers: environment capture, core pinning, statistics, UR3 DH kinematics, input generation."""
import json
import math
import os
import platform
import shutil
import subprocess
import time

import numpy as np

HOME = os.path.expanduser("~")
UR3_XML = os.environ.get("CTRLBENCH_UR3_XML", os.path.join(HOME, "UR3_ROS2_PICK_AND_PLACE/ur_rl_training/models/ur3/ur3.xml"))
Q_HOME = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# UR3 standard DH parameters (m, rad)
DH_A = np.array([0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0])
DH_D = np.array([0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819])
DH_ALPHA = np.array([math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0])

# IK settings, identical in every implementation
IK_LAMBDA, IK_TOL, IK_MAX_ITER = 0.05, 1e-4, 200
# Impedance gains, identical in every implementation
K_GAIN = np.array([800.0, 800.0, 800.0, 30.0, 30.0, 30.0])
D_GAIN = np.array([40.0, 40.0, 40.0, 2.0, 2.0, 2.0])


# ---------- environment ----------
def _cmd(c):
    try:
        return subprocess.run(c, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:
        return ""


def cpu_utilisation(cpus=None, interval=1.0):
    """Per-cpu busy fraction over `interval` seconds, from /proc/stat."""
    def snap():
        out = {}
        for line in open("/proc/stat"):
            if line.startswith("cpu") and line[3].isdigit():
                f = line.split()
                v = list(map(int, f[1:]))
                idle = v[3] + v[4]
                out[int(f[0][3:])] = (sum(v), idle)
        return out
    a = snap()
    time.sleep(interval)
    b = snap()
    return {c: 1.0 - (b[c][1] - a[c][1]) / max(1, (b[c][0] - a[c][0])) for c in b}


def pick_core(candidates=range(0, 12), exclude=()):
    """Least busy cpu among `candidates` (the first 12 logical cpus are the P-cores on this machine)."""
    util = cpu_utilisation()
    cs = [c for c in candidates if c in util and c not in exclude]
    return min(cs, key=lambda c: util[c]), util


def env_info():
    cpu = ""
    for line in open("/proc/cpuinfo"):
        if line.startswith("model name"):
            cpu = line.split(":", 1)[1].strip()
            break
    gov = ""
    try:
        gov = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read().strip()
    except OSError:
        pass
    import mujoco
    import pinocchio
    info = dict(
        cpu=cpu, logical_cpus=os.cpu_count(), governor=gov, kernel=platform.release(),
        isolcpus="isolcpus" in open("/proc/cmdline").read(),
        loadavg_at_start=os.getloadavg(), python=platform.python_version(), numpy=np.__version__,
        mujoco=mujoco.__version__, pinocchio=pinocchio.__version__,
        gcc=_cmd(["g++", "--version"]).splitlines()[0] if shutil.which("g++") else "",
        cmake=_cmd(["cmake", "--version"]).splitlines()[0] if shutil.which("cmake") else "",
        ros_distro=os.environ.get("ROS_DISTRO", ""), taskset=bool(shutil.which("taskset")),
        date=time.strftime("%Y-%m-%d %H:%M:%S"),
    )
    return info


def save_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)


# ---------- statistics ----------
def stats(x):
    x = np.asarray(x, dtype=float)
    p50, p95, p99 = np.percentile(x, [50, 95, 99])
    q1, q3 = np.percentile(x, [25, 75])
    return dict(n=len(x), median=p50, p95=p95, p99=p99, mean=x.mean(), std=x.std(), iqr=q3 - q1,
                jitter=p99 - p50, max=x.max())


# ---------- UR3 DH kinematics (numpy) ----------
def dh(th, d, a, al):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_frames(q):
    T = [np.eye(4)]
    for i in range(6):
        T.append(T[-1] @ dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i]))
    return T


def fk(q):
    T = fk_frames(q)[-1]
    return T[:3, :3], T[:3, 3]


def rot_err(Rc, Rt):
    return 0.5 * (np.cross(Rc[:, 0], Rt[:, 0]) + np.cross(Rc[:, 1], Rt[:, 1]) + np.cross(Rc[:, 2], Rt[:, 2]))


def jac_dh(T):
    pe = T[6][:3, 3]
    J = np.empty((6, 6))
    for i in range(6):
        z, p = T[i][:3, 2], T[i][:3, 3]
        J[:3, i] = np.cross(z, pe - p)
        J[3:, i] = z
    return J


def dls_step(J, e):
    return J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA ** 2 * np.eye(6), e)


# ---------- inputs ----------
def make_ik_targets(n=500, seed=0):
    """Target poses from FK of random joint vectors near the home pose (always reachable)."""
    rng = np.random.default_rng(seed)
    q = Q_HOME + rng.normal(0, 0.6, (n, 6))
    rows = []
    for qi in q:
        R, p = fk(qi)
        rows.append(np.concatenate([p, R.ravel(), qi]))
    return np.array(rows)


def make_mj_targets(n=500, seed=0, xml=UR3_XML):
    import mujoco
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    rng = np.random.default_rng(seed)
    q = Q_HOME + rng.normal(0, 0.6, (n, 6))
    rows = []
    for qi in q:
        d.qpos[:] = qi
        mujoco.mj_kinematics(m, d)
        rows.append(np.concatenate([d.site_xpos[sid], d.site_xmat[sid], qi]))
    return np.array(rows)


def make_states(n=2000, seed=1, xml=UR3_XML):
    """Impedance inputs: q, v, J(36), g, e(6), x_des(3), R_des(9). J, g, e come from Pinocchio."""
    import pinocchio as pin
    m = pin.buildModelFromMJCF(xml)
    d = m.createData()
    fid = m.getFrameId("attachment_site")
    rng = np.random.default_rng(seed)
    rows = []
    for _ in range(n):
        q = Q_HOME + rng.normal(0, 0.6, 6)
        v = rng.normal(0, 0.2, 6)
        qd = q + rng.normal(0, 0.05, 6)                      # desired pose = FK of a nearby configuration
        pin.framesForwardKinematics(m, d, qd)
        Rd, xd = d.oMf[fid].rotation.copy(), d.oMf[fid].translation.copy()
        pin.framesForwardKinematics(m, d, q)
        R, x = d.oMf[fid].rotation, d.oMf[fid].translation
        J = pin.computeFrameJacobian(m, d, q, fid, pin.LOCAL_WORLD_ALIGNED)
        g = pin.computeGeneralizedGravity(m, d, q)
        e = np.concatenate([xd - x, pin.log3(Rd @ R.T)])
        rows.append(np.concatenate([q, v, J.ravel(), g, e, xd, Rd.ravel()]))
    return np.array(rows)


def write_csv(path, arr):
    np.savetxt(path, arr, delimiter=",", fmt="%.17g")
