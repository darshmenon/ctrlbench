"""Run every benchmark R times, pinned to the least busy core, Python and C++ interleaved, and write results/.

  python -m ctrlbench.run_all --build build --repeats 5

Needs the C++ build (`make build`) and, for the ROS 2 benchmark, `source /opt/ros/humble/setup.bash`.
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import time

import numpy as np

from ctrlbench import common as C

BENCHES = [("ik_dh", "IK, UR3 DH model", "ik_targets.csv"), ("ik_mj", "IK, MuJoCo model", "mj_targets.csv"),
           ("imp_law", "Impedance law only", "states.csv"), ("imp_pin", "Impedance with Pinocchio", "states.csv")]


def run_pinned(core, cmd, env=None, **kw):
    if C.shutil.which("taskset"):
        cmd = ["taskset", "-c", str(core)] + cmd
    return subprocess.run(cmd, check=True, env=env, **kw)


def cpp_cmd(b, binname, name, data, out):
    if name == "ik_dh":
        return [f"{b}/ik_dh", f"{data}/ik_targets.csv", out]
    if name == "ik_mj":
        return [f"{b}/ik_mj", C.UR3_XML, f"{data}/mj_targets.csv", out]
    mode = "law" if name == "imp_law" else "pin"
    return [f"{b}/imp", mode, C.UR3_XML, f"{data}/states.csv", out]


def cpu_ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])            # utime + stime


def rss_mb(pid):
    for line in open(f"/proc/{pid}/status"):
        if line.startswith("VmRSS"):
            return int(line.split()[1]) / 1024


def run_rcl(b, impl, core_dut, core_h, secs, rate, out):
    env = dict(os.environ, ROS_DOMAIN_ID="88")
    pre = ["taskset", "-c", str(core_dut)] if C.shutil.which("taskset") else []
    hpre = ["taskset", "-c", str(core_h)] if C.shutil.which("taskset") else []
    cmd = [f"{b}/rcl_dut"] if impl == "cpp" else [sys.executable, "-m", "ctrlbench.rcl_dut"]
    dut = subprocess.Popen(pre + cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(3.0)
        # harness prints once the timed window is over; sample the DUT's CPU time around it
        h = subprocess.Popen(hpre + [f"{b}/rcl_harness", str(rate), str(secs), out], env=env, stdout=subprocess.PIPE, text=True)
        time.sleep(4.0)                       # discovery + the harness' own settle time
        t0, c0 = time.time(), cpu_ticks(dut.pid)
        h.wait(timeout=secs + 60)
        t1, c1 = time.time(), cpu_ticks(dut.pid)
        cpu_pct = 100.0 * (c1 - c0) / os.sysconf("SC_CLK_TCK") / (t1 - t0)
        rss = rss_mb(dut.pid)
        line = h.stdout.read().strip()
    finally:
        dut.send_signal(signal.SIGINT)
        try:
            dut.wait(timeout=10)
        except subprocess.TimeoutExpired:
            dut.kill()
    kv = dict(p.split("=") for p in line.split() if "=" in p)
    return dict(cpu_pct=cpu_pct, rss_mb=rss, achieved_rate=float(kv.get("achieved_rate", 0)),
                published=int(kv.get("published", 0)), received=int(kv.get("received", 0)))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default="build")
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--n-ik", type=int, default=500)
    ap.add_argument("--n-states", type=int, default=2000)
    ap.add_argument("--rcl-secs", type=float, default=20)
    ap.add_argument("--rate", type=float, default=500)
    ap.add_argument("--out", default="results")
    ap.add_argument("--skip-rcl", action="store_true")
    a = ap.parse_args(argv)
    os.makedirs(f"{a.out}/raw", exist_ok=True)
    os.makedirs("data", exist_ok=True)
    C.write_csv("data/ik_targets.csv", C.make_ik_targets(a.n_ik))
    C.write_csv("data/mj_targets.csv", C.make_mj_targets(a.n_ik))
    C.write_csv("data/states.csv", C.make_states(a.n_states))

    from ctrlbench.bench_py import timer_overhead
    env = C.env_info()
    env["timer_overhead_ns"] = dict(python=timer_overhead(),
                                    cpp=float(subprocess.run([f"{a.build}/timer_overhead"], capture_output=True, text=True).stdout))
    env["settings"] = dict(repeats=a.repeats, n_ik=a.n_ik, n_states=a.n_states, rcl_secs=a.rcl_secs, rate_hz=a.rate)
    runs = []
    rows = []
    have_rcl = os.path.exists(f"{a.build}/rcl_harness") and not a.skip_rcl and os.environ.get("ROS_DISTRO")
    for rep in range(a.repeats):
        core, util = C.pick_core()
        info = dict(repeat=rep, core=core, core_util_before=round(util[core], 3),
                    machine_util_before=round(float(np.mean(list(util.values()))), 3), loadavg=os.getloadavg()[0])
        print(f"repeat {rep}: core {core} (busy {util[core]:.0%}), load {info['loadavg']:.1f}", flush=True)
        for name, label, inp in BENCHES:
            order = ["py", "cpp"] if rep % 2 == 0 else ["cpp", "py"]
            for impl in order:
                out = f"{a.out}/raw/{name}_{impl}_{rep}.csv"
                if impl == "py":
                    run_pinned(core, [sys.executable, "-m", "ctrlbench.bench_py", name, f"data/{inp}", out])
                else:
                    run_pinned(core, cpp_cmd(a.build, None, name, "data", out))
                x = np.loadtxt(out, delimiter=",", skiprows=1)
                s = C.stats(x[:, 0] / 1e3)                      # microseconds
                s.update(bench=name, impl=impl, repeat=rep, mean_iters=float(x[:, 1].mean()))
                rows.append(s)
        if have_rcl:
            core_d, util = C.pick_core()
            core_h, _ = C.pick_core(exclude=(core_d,))
            for impl in (["py", "cpp"] if rep % 2 == 0 else ["cpp", "py"]):
                out = f"{a.out}/raw/rcl_{impl}_{rep}.csv"
                r = run_rcl(a.build, impl, core_d, core_h, a.rcl_secs, a.rate, out)
                x = np.loadtxt(out, delimiter=",", skiprows=1)
                s = C.stats(x)
                s.update(bench="rcl", impl=impl, repeat=rep, mean_iters=0.0, **r)
                rows.append(s)
                print(f"  rcl {impl}: median {s['median']:.0f} us, p99 {s['p99']:.0f} us, cpu {r['cpu_pct']:.1f}%, rate {r['achieved_rate']:.0f}", flush=True)
        info["loadavg_end"] = os.getloadavg()[0]
        runs.append(info)
    env["loadavg_at_end"] = os.getloadavg()
    C.save_json(f"{a.out}/env.json", env)
    C.save_json(f"{a.out}/runs.json", runs)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ("bench", "impl", "repeat", "n"), k))
    with open(f"{a.out}/summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    from ctrlbench import report
    report.main(["--out", a.out])


if __name__ == "__main__":
    main()
