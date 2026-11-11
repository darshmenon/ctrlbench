"""Aggregate results/summary.csv into results/aggregate.csv, results/table.md and the plots."""
import argparse
import csv
import json
import os

import numpy as np

LABEL = {"ik_dh": "IK, UR3 DH model", "ik_mj": "IK, MuJoCo model", "imp_law": "Impedance law only",
         "imp_pin": "Impedance with Pinocchio", "rcl": "ROS 2 round trip at 500 Hz"}
UNIT = {"rcl": "latency"}


def load(out):
    rows = list(csv.DictReader(open(f"{out}/summary.csv")))
    for r in rows:
        for k, v in r.items():
            if k not in ("bench", "impl"):
                r[k] = float(v) if v != "" else float("nan")
    return rows


def per_iter_us(out, bench, impl):
    """Median time per IK iteration (time / (iterations + 1)), pooled over repeats."""
    vals = []
    for f in sorted(os.listdir(f"{out}/raw")):
        if f.startswith(f"{bench}_{impl}_"):
            x = np.loadtxt(f"{out}/raw/{f}", delimiter=",", skiprows=1)
            vals.append(x[:, 0] / 1e3 / (x[:, 1] + 1))
    return float(np.median(np.concatenate(vals))) if vals else float("nan")


def aggregate(rows, out="results"):
    agg = []
    for bench in LABEL:
        for impl in ("py", "cpp"):
            rs = [r for r in rows if r["bench"] == bench and r["impl"] == impl]
            if not rs:
                continue
            med = np.array([r["median"] for r in rs])
            a = dict(bench=bench, impl=impl, repeats=len(rs), median_us=float(np.median(med)),
                     median_min_us=float(med.min()), median_max_us=float(med.max()),
                     median_cv_pct=float(100 * med.std() / med.mean()),
                     p95_us=float(np.median([r["p95"] for r in rs])), p99_us=float(np.median([r["p99"] for r in rs])),
                     jitter_us=float(np.median([r["jitter"] for r in rs])), iqr_us=float(np.median([r["iqr"] for r in rs])))
            if bench in ("ik_dh", "ik_mj"):
                a["per_iter_us"] = per_iter_us(out, bench, impl)
            if bench == "rcl":
                a["cpu_pct"] = float(np.median([r["cpu_pct"] for r in rs]))
                a["rss_mb"] = float(np.median([r["rss_mb"] for r in rs]))
                a["achieved_rate_hz"] = float(np.median([r["achieved_rate"] for r in rs]))
                a["loss_pct"] = float(np.median([max(0.0, 100 * (1 - r["received"] / max(1, r["published"]))) for r in rs]))
            agg.append(a)
    return agg


def fmt(x):
    return f"{x:,.2f}" if x < 100 else f"{x:,.0f}"


def table(agg, env):
    lines = ["| Benchmark | Python median | C++ median | Python p99 | C++ p99 | Python / C++ (median) | Spread of repeat medians (py, C++) |",
             "|---|---|---|---|---|---|---|"]
    for bench in LABEL:
        p = next((a for a in agg if a["bench"] == bench and a["impl"] == "py"), None)
        c = next((a for a in agg if a["bench"] == bench and a["impl"] == "cpp"), None)
        if not p or not c:
            continue
        lines.append(f"| {LABEL[bench]} | {fmt(p['median_us'])} us | {fmt(c['median_us'])} us | {fmt(p['p99_us'])} us | "
                     f"{fmt(c['p99_us'])} us | {p['median_us'] / c['median_us']:.1f}x | "
                     f"{fmt(p['median_min_us'])} to {fmt(p['median_max_us'])} us, {fmt(c['median_min_us'])} to {fmt(c['median_max_us'])} us |")
    its = [(b, next(a for a in agg if a["bench"] == b and a["impl"] == "py"), next(a for a in agg if a["bench"] == b and a["impl"] == "cpp"))
           for b in ("ik_dh", "ik_mj") if any(a["bench"] == b for a in agg)]
    if its:
        lines += ["", "| IK cost per iteration | Python | C++ | Python / C++ |", "|---|---|---|---|"]
        for b, p, c in its:
            lines.append(f"| {LABEL[b]} | {fmt(p['per_iter_us'])} us | {fmt(c['per_iter_us'])} us | {p['per_iter_us'] / c['per_iter_us']:.1f}x |")
    rc = [a for a in agg if a["bench"] == "rcl"]
    extra = ""
    if rc:
        extra = "\n\n| ROS 2 node | CPU (one core) | Memory (RSS) | Achieved publish rate | Messages lost |\n|---|---|---|---|---|\n"
        for a in rc:
            extra += f"| {'rclpy' if a['impl'] == 'py' else 'rclcpp'} | {a['cpu_pct']:.1f}% | {a['rss_mb']:.0f} MB | {a['achieved_rate_hz']:.0f} Hz | {a['loss_pct']:.2f}% |\n"
    return "\n".join(lines) + extra


def plots(out, agg):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"py": "#3b7dd8", "cpp": "#d8693b"}
    name = {"py": "Python", "cpp": "C++"}
    benches = [b for b in LABEL if any(a["bench"] == b for a in agg)]
    # bar: median with the p99 as the whisker top, log scale
    fig, ax = plt.subplots(figsize=(10, 4.6))
    w = 0.38
    for j, impl in enumerate(("py", "cpp")):
        xs, med, p99 = [], [], []
        for i, b in enumerate(benches):
            a = next((a for a in agg if a["bench"] == b and a["impl"] == impl), None)
            if a:
                xs.append(i + (j - 0.5) * w); med.append(a["median_us"]); p99.append(a["p99_us"])
        bars = ax.bar(xs, med, w, color=col[impl], label=name[impl])
        ax.errorbar(xs, med, yerr=[np.zeros(len(med)), np.array(p99) - np.array(med)], fmt="none", ecolor="k", capsize=3, lw=1)
        for x, m in zip(xs, med):
            ax.text(x, m * 1.15, fmt(m), ha="center", va="bottom", fontsize=8)
    ax.set_yscale("log")
    ax.set_xticks(range(len(benches)))
    ax.set_xticklabels([LABEL[b].replace(", ", ",\n").replace(" at ", "\nat ") for b in benches], fontsize=9)
    ax.set_ylabel("time per call, microseconds (log scale)")
    ax.set_title("C++ vs Python: median per call, whisker up to p99")
    ax.legend()
    ax.grid(axis="y", alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(f"{out}/bars.png", dpi=140)
    plt.close(fig)
    # distributions, pooled over repeats
    n = len(benches)
    fig, axs = plt.subplots(1, n, figsize=(3.6 * n, 3.6))
    for ax, b in zip(np.atleast_1d(axs), benches):
        series = {}
        for impl in ("py", "cpp"):
            vals = []
            for f in sorted(os.listdir(f"{out}/raw")):
                if f.startswith(f"{b}_{impl}_"):
                    x = np.loadtxt(f"{out}/raw/{f}", delimiter=",", skiprows=1)
                    vals.append(x if b == "rcl" else x[:, 0] / 1e3)
            if vals:
                series[impl] = np.concatenate(vals) if b == "rcl" else np.concatenate(vals)
        if not series:
            continue
        lo = min(v.min() for v in series.values())
        hi = max(np.percentile(v, 99.9) for v in series.values())
        bins = np.logspace(np.log10(max(lo, 1e-2)), np.log10(hi), 60)
        for impl, v in series.items():
            ax.hist(v, bins=bins, alpha=0.65, color=col[impl], label=name[impl])
        ax.set_xscale("log")
        ax.set_title(LABEL[b], fontsize=9)
        ax.set_xlabel("microseconds")
    np.atleast_1d(axs)[0].set_ylabel("calls")
    np.atleast_1d(axs)[0].legend()
    fig.tight_layout()
    fig.savefig(f"{out}/distributions.png", dpi=140)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results")
    a = ap.parse_args(argv)
    rows = load(a.out)
    agg = aggregate(rows, a.out)
    env = json.load(open(f"{a.out}/env.json"))
    with open(f"{a.out}/aggregate.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted({k for r in agg for k in r}, key=lambda k: (k not in ("bench", "impl"), k)))
        w.writeheader()
        w.writerows(agg)
    open(f"{a.out}/table.md", "w").write(table(agg, env) + "\n")
    plots(a.out, agg)
    print(table(agg, env))


if __name__ == "__main__":
    main()
