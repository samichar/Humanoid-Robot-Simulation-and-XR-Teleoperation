r"""Replay an XR recording through the simulator-side input pipeline.

Pipeline:  recording (SteamVR axes, xyzw, ~90 Hz, irregular)
        -> frames.py axis/quaternion conversion
        -> calibration (heading, origin, floor, scale) from the neutral window
        -> resample to CONTROL_HZ (linear position, slerp orientation, gaps flagged)
        -> drive mocap target markers in the G1 scene, robot holds its stand pose
        -> log targets + robot state at CONTROL_HZ

Calibration here is a PLACEHOLDER with the same interface Task 2 will deliver
(heading, origin, floor, scale). Swap in Task 2's calibration file later.

Usage:
  python replay.py                              # mock recording, headless, validates vs truth
  python replay.py --view                       # same, in the viewer (real time)
  python replay.py --recording ..\data\real.csv  # a real recording (no truth check)
"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np
import pandas as pd
import mujoco
from scipy.spatial.transform import Rotation as R, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.frames import steam_pos_to_mj, steam_quat_to_mj, xyzw_to_wxyz
from config import SCENE_XML, KEYFRAME, CONTROL_HZ, RESULTS, ROOT

NEUTRAL_S = 1.5      # first seconds of every recording: operator stands in neutral pose
MAX_GAP_S = 0.05     # resampled points inside a longer gap are marked invalid, not interpolated
COLORS = {"head": [1, 1, 0, 1], "hand_left": [0, 1, 0, 1], "hand_right": [1, 0, 0, 1],
          "pelvis": [0, 0.6, 1, 1], "foot_left": [1, 0, 1, 1], "foot_right": [1, 0.5, 0, 1]}


def load_recording(path):
    df = pd.read_csv(path)
    out = {}
    for seg, g in df.groupby("segment"):
        g = g.sort_values("t_source")
        out[seg] = {"t": g.t_source.to_numpy(),
                    "p": steam_pos_to_mj(g[["px", "py", "pz"]].to_numpy()),
                    "q": steam_quat_to_mj(g[["qx", "qy", "qz", "qw"]].to_numpy()),
                    "valid": g.valid.to_numpy().astype(bool)}
    t0 = min(d["t"][0] for d in out.values())
    for d in out.values():
        d["t"] = d["t"] - t0                       # common time origin
    return out

def load_npz_recording(path):
    """Load a Task 3 recording (src/record.py format: <id>.npz + <id>.json).
    Positions in OpenVR axes, quaternions wxyz; invalid samples are NaN and dropped here."""
    base = Path(path).with_suffix("")
    z = np.load(f"{base}.npz"); meta = json.loads(Path(f"{base}.json").read_text())
    role = {d["name"]: d["role"] for d in meta["device_table"]}
    t = z["t_system"]; out = {}
    for name in meta["devices"]:
        p = z[f"{name}__position"]; q = z[f"{name}__quat_wxyz"]
        ok = z[f"{name}__valid"].astype(bool) & np.isfinite(p).all(1) & np.isfinite(q).all(1)
        if ok.sum() < 2:
            print(f"[warn] {name}: fewer than 2 valid samples, skipped"); continue
        out[role[name]] = {"t": t[ok] - t[0],
                           "p": steam_pos_to_mj(p[ok]),
                           "q": steam_quat_to_mj(q[ok][:, [1, 2, 3, 0]]),   # wxyz -> xyzw
                           "valid": np.ones(ok.sum(), bool)}
    return out

def calibrate(rec, robot_pelvis_z):
    """Placeholder calibration from the neutral window. Returns a dict Task 2 can replace."""
    def neutral(seg):
        d = rec[seg]; m = (d["t"] < NEUTRAL_S) & d["valid"]
        return d["p"][m], d["q"][m]
    p_head, q_head = neutral("head"); p_pel, _ = neutral("pelvis")
    fwd = R.from_quat(q_head).apply([1, 0, 0]).mean(axis=0)   # head forward, MuJoCo axes
    yaw = float(np.arctan2(fwd[1], fwd[0]))
    origin = np.array([p_pel[:, 0].mean(), p_pel[:, 1].mean(), 0.0])  # floor at z=0 (standing universe)
    scale = float(robot_pelvis_z / p_pel[:, 2].mean())
    return {"yaw_rad": yaw, "origin_m": origin.tolist(), "floor_m": 0.0, "scale": scale}


def apply_calibration(rec, cal):
    Rinv = R.from_euler("z", -cal["yaw_rad"]); o = np.array(cal["origin_m"])
    for d in rec.values():
        d["p"] = cal["scale"] * Rinv.apply(d["p"] - o)
        d["q"] = (Rinv * R.from_quat(d["q"])).as_quat()


def resample(rec, hz):
    t_end = min(d["t"][-1] for d in rec.values())
    grid = np.arange(0, t_end, 1 / hz); out = {"t": grid}
    for seg, d in rec.items():
        t, p, q = d["t"][d["valid"]], d["p"][d["valid"]], d["q"][d["valid"]]
        idx = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
        gap = t[idx] - t[idx - 1]
        ok = (gap <= MAX_GAP_S) & (grid >= t[0]) & (grid <= t[-1])
        pos = np.stack([np.interp(grid, t, p[:, k]) for k in range(3)], axis=1)
        quat = Slerp(t, R.from_quat(q))(np.clip(grid, t[0], t[-1])).as_quat()
        out[seg] = {"p": pos, "q": quat, "valid": ok}
    return out


def build_scene(segments):
    spec = mujoco.MjSpec.from_file(str(SCENE_XML))
    for seg in segments:
        b = spec.worldbody.add_body(name=f"target_{seg}", mocap=True)
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.035, 0, 0],
                   contype=0, conaffinity=0, rgba=COLORS.get(seg, [1, 1, 1, 1]))
    return spec.compile()


def main(args):
    rec = load_recording(args.recording)
    M = build_scene(sorted(rec)); D = mujoco.MjData(M)
    mujoco.mj_resetDataKeyframe(M, D, mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_KEY, KEYFRAME))
    cal = calibrate(rec, robot_pelvis_z=float(D.qpos[2]))
    apply_calibration(rec, cal)
    rs = resample(rec, CONTROL_HZ)
    segs = sorted(rec)
    mocap = {s: M.body_mocapid[mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_BODY, f"target_{s}")] for s in segs}
    sub = int(round(1 / (CONTROL_HZ * M.opt.timestep)))
    last = {}; log_q = []

    def step(k):
        for s in segs:
            if rs[s]["valid"][k]:
                last[s] = (rs[s]["p"][k], xyzw_to_wxyz(rs[s]["q"][k]))
            if s in last:                                   # invalid sample: hold last valid target
                D.mocap_pos[mocap[s]], D.mocap_quat[mocap[s]] = last[s]
        for _ in range(sub):
            mujoco.mj_step(M, D)
        log_q.append(D.qpos.copy())

    if args.view:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(M, D) as v:
            for k in range(len(rs["t"])):
                if not v.is_running(): break
                t0 = time.perf_counter(); step(k); v.sync()
                time.sleep(max(0, 1 / CONTROL_HZ - (time.perf_counter() - t0)))
    else:
        for k in range(len(rs["t"])): step(k)

    RESULTS.mkdir(exist_ok=True)
    np.savez(RESULTS / "replay_log.npz", t=rs["t"], qpos=np.array(log_q),
             **{f"{s}_{f}": rs[s][f] for s in segs for f in ("p", "q", "valid")})
    report = {"recording": str(args.recording), "control_hz": CONTROL_HZ,
              "steps": len(rs["t"]), "calibration": cal,
              "invalid_steps": {s: int((~rs[s]["valid"]).sum()) for s in segs}}

    truth_path = Path(args.recording).with_name("mock_truth.csv")
    if truth_path.exists() and "mock" in Path(args.recording).name:
        tr = pd.read_csv(truth_path); report["truth_check"] = {}
        for s in segs:
            v = rs[s]["valid"]
            exp = cal["scale"] * np.stack([np.interp(rs["t"], tr.t, tr[f"{s}_p{c}"]) for c in "xyz"], 1)
            err = np.linalg.norm(rs[s]["p"][v] - exp[v], axis=1) * 1000
            qt = R.from_quat(np.stack([np.interp(rs["t"], tr.t, tr[f"{s}_q{c}"]) for c in "xyzw"], 1))
            ang = np.degrees((R.from_quat(rs[s]["q"][v]) * qt[v].inv()).magnitude())
            report["truth_check"][s] = {"pos_rms_mm": float(np.sqrt((err**2).mean())),
                                        "pos_max_mm": float(err.max()), "ang_max_deg": float(ang.max())}
        worst = max(c["pos_max_mm"] for c in report["truth_check"].values())
        report["truth_check_passed"] = bool(worst < 10.0)

    (RESULTS / "replay_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--recording", default=str(ROOT / "data" / "mock" / "mock_session.csv"))
    ap.add_argument("--view", action="store_true")
    main(ap.parse_args())
