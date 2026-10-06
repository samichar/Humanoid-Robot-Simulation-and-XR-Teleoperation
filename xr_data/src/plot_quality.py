"""quality plots and machine-readable summary for one recording.
python src/plot_quality.py data/raw/x.npz --outdir plots
produces: <id>_rate.png  <id>_gaps.png  <id>_ranges.png  <id>_trajectories.png  <id>_quality.json
"""
import argparse, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from io_utils import load_recording


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("--outdir", default="plots")
    ap.add_argument("--gap-factor", type=float, default=1.5, help="dt > factor*nominal counts as a dropped tick")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    t, data, meta = load_recording(a.inp)
    rid = meta["recording_id"]; names = list(data); nominal = 1.0 / meta["target_rate_hz"]
    if meta.get("processed"):
        nominal = 1.0 / meta["processing"]["rate_hz"]
    dt = np.diff(t)
    dropped_ticks = int(np.sum(np.round(dt[dt > a.gap_factor * nominal] / nominal) - 1))
    summary = {"recording_id": rid, "sequence": meta["sequence"], "duration_s": float(t[-1] - t[0]),
               "n_samples": int(len(t)), "mean_rate_hz": float(1 / dt.mean()), "dt_std_ms": float(dt.std() * 1e3),
               "dt_max_ms": float(dt.max() * 1e3), "dropped_ticks": dropped_ticks, "devices": {}}

    # 1. sample rate
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].hist(dt * 1e3, bins=60, color="#3a6ea5"); ax[0].set_yscale("log")
    ax[0].axvline(nominal * 1e3, color="r", ls="--", label=f"nominal {nominal*1e3:.1f} ms"); ax[0].legend()
    ax[0].set_xlabel("inter-sample interval (ms)"); ax[0].set_ylabel("count (log)"); ax[0].set_title("Interval histogram")
    win = max(1, int(1 / nominal))
    inst = 1 / dt; roll = np.convolve(inst, np.ones(win) / win, mode="same")
    ax[1].plot(t[1:], roll, lw=1); ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("sample rate (Hz), 1 s moving avg")
    ax[1].set_title("Rate vs time")
    fig.suptitle(f"{rid} ({meta['sequence']}), N={len(t)} samples"); fig.tight_layout()
    fig.savefig(f"{a.outdir}/{rid}_rate.png", dpi=130); plt.close(fig)

    # 2. gaps / invalid frames
    fig, ax = plt.subplots(figsize=(11, 0.9 + 0.55 * len(names)))
    for i, n in enumerate(names):
        inv = ~data[n]["valid"]
        ax.scatter(t[inv], np.full(inv.sum(), i), marker="|", s=120, color="crimson")
        summary["devices"][n] = {"invalid_frames": int(inv.sum()), "invalid_pct": float(inv.mean() * 100)}
    big = np.flatnonzero(dt > a.gap_factor * nominal)
    for b in big:
        ax.axvspan(t[b], t[b + 1], color="orange", alpha=0.5, lw=0)
    ax.set_yticks(range(len(names))); ax.set_yticklabels(names); ax.set_xlim(t[0], t[-1])
    ax.set_xlabel("time (s)"); ax.set_title(f"Invalid frames (red) and dropped ticks (orange, {dropped_ticks} ticks lost)")
    fig.tight_layout(); fig.savefig(f"{a.outdir}/{rid}_gaps.png", dpi=130); plt.close(fig)

    # 3. pose ranges
    fig, ax = plt.subplots(figsize=(10, 4)); w = 0.8 / 3
    for k, ax_name in enumerate("xyz"):
        lo = [np.nanmin(data[n]["position"][:, k]) for n in names]
        hi = [np.nanmax(data[n]["position"][:, k]) for n in names]
        ax.bar(np.arange(len(names)) + k * w, np.array(hi) - np.array(lo), w, bottom=lo, label=ax_name)
        for n, l, h in zip(names, lo, hi):
            summary["devices"][n][f"range_{ax_name}_m"] = [float(l), float(h)]
    ax.set_xticks(np.arange(len(names)) + w); ax.set_xticklabels(names); ax.set_ylabel("position range (m)")
    ax.set_title("Per-axis min-max position (floating bars)"); ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(f"{a.outdir}/{rid}_ranges.png", dpi=130); plt.close(fig)

    # 4. synchronized trajectories
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for k, lab in enumerate(["x (m)", "y, up (m)", "z (m)"]):
        for n in names:
            ax[k].plot(t, data[n]["position"][:, k], lw=1, label=n)
        ax[k].set_ylabel(lab); ax[k].grid(alpha=.3)
    ax[0].legend(ncol=len(names), fontsize=8); ax[-1].set_xlabel("time (s)")
    fig.suptitle(f"Synchronized trajectories, {rid}"); fig.tight_layout()
    fig.savefig(f"{a.outdir}/{rid}_trajectories.png", dpi=130); plt.close(fig)

    json.dump(summary, open(f"{a.outdir}/{rid}_quality.json", "w"), indent=2)
    print(json.dumps({k: v for k, v in summary.items() if k != "devices"}, indent=1))
    for n, d in summary["devices"].items():
        print(f"  {n:12s} invalid {d['invalid_pct']:.1f}%")


if __name__ == "__main__":
    main()
