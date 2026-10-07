"""Generate a synthetic XR recording in the Task 3 schema, with known ground truth.

The operator performs: neutral stand (0-2 s), forward arm raise (2-5 s),
squat (5-8 s), left-foot step (8-10 s). The operator stands at an arbitrary
offset and heading inside the tracking space, as a real user would.

Imperfections injected (so the pipeline's handling gets exercised):
  ~90 Hz with timestamp jitter, 1 mm position noise, a 150 ms dropout on the
  foot tracker (rows missing), and 10 frames flagged valid=0 on the left hand.

Outputs (data/mock/):
  mock_session.csv  recording: t_source, device_serial, segment, px..pz, qx..qw, valid
                    (SteamVR axes, quaternion xyzw)
  mock_truth.csv    ground truth at 1 kHz in the OPERATOR frame (MuJoCo axes,
                    origin on the floor under the neutral pelvis)

Usage: python tools/mock_xr.py [--seed 0]
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.frames import mj_pos_to_steam, mj_quat_to_steam

DEVICES = {  # segment: (serial, neutral position in operator frame, m)
    "head":       ("LHR-HMD00001", np.array([0.00, 0.00, 1.70])),
    "left_hand":  ("LHR-CTRL0001", np.array([0.00, 0.25, 0.95])),
    "right_hand": ("LHR-CTRL0002", np.array([0.00, -0.25, 0.95])),
    "pelvis":     ("LHR-TRKR0001", np.array([0.00, 0.00, 1.00])),
    "right_foot":  ("LHR-TRKR0002", np.array([0.00, 0.10, 0.08])),
    "left_foot":  ("LHR-TRKR0003", np.array([0.00, 0.10, 0.08])),
}
DURATION = 10.0


def smooth(t, t0, t1):
    """0 before t0, 1 at the midpoint, back to 0 at t1 (raised-cosine bump)."""
    x = np.clip((t - t0) / (t1 - t0), 0, 1)
    return 0.5 * (1 - np.cos(2 * np.pi * x))


def operator_motion(seg, t):
    """Ground-truth pose of one segment in the operator frame. Returns (pos, quat_xyzw)."""
    p = np.tile(DEVICES[seg][1], (len(t), 1)).astype(float)
    pitch = np.zeros_like(t)
    raise_ = smooth(t, 2, 5); squat = smooth(t, 5, 8); step = smooth(t, 8, 10)
    if seg.startswith("hand"):
        p[:, 0] += 0.45 * raise_; p[:, 2] += 0.40 * raise_       # arms forward and up
        pitch = -np.deg2rad(60) * raise_
    if seg in ("head", "pelvis", "hand_left", "hand_right"):
        p[:, 2] -= 0.25 * squat                                    # whole upper body lowers
    if seg == "left_foot":
        p[:, 2] += 0.15 * smooth(t, 8, 9)                          # left foot steps first
    if seg == "right_foot":
        p[:, 2] += 0.15 * smooth(t, 9, 10)                         # then right foot                                    # foot lifts
    q = R.from_euler("y", pitch[:, None]).as_quat()
    return p, q


def main(seed):
    rng = np.random.default_rng(seed)
    yaw0 = np.deg2rad(37.0)                       # operator faces 37 deg off tracking-forward
    t_offset = np.array([1.2, -0.6, 0.0])         # operator stands away from tracking origin
    Rz = R.from_euler("z", yaw0)

    rows = []
    t_nom = np.arange(0, DURATION, 1 / 90)
    for seg, (serial, _) in DEVICES.items():
        t = 1000.0 + t_nom + rng.normal(0, 0.001, t_nom.size)   # arbitrary clock origin, jitter
        p_op, q_op = operator_motion(seg, t - 1000.0)
        p_trk = Rz.apply(p_op) + t_offset                          # MuJoCo-axis tracking space
        q_trk = (Rz * R.from_quat(q_op)).as_quat()
        p_steam = mj_pos_to_steam(p_trk) + rng.normal(0, 0.001, p_trk.shape)
        q_steam = mj_quat_to_steam(q_trk)
        valid = np.ones(t.size, dtype=int)
        keep = np.ones(t.size, dtype=bool)
        if seg == "foot_left":
            keep &= ~((t - 1000.0 > 6.0) & (t - 1000.0 < 6.15))     # 150 ms dropout
        if seg == "hand_left":
            valid[(t - 1000.0 > 4.0) & (t - 1000.0 < 4.11)] = 0      # flagged invalid
        for i in np.flatnonzero(keep):
            rows.append([t[i], serial, seg, *p_steam[i], *q_steam[i], valid[i]])

    df = pd.DataFrame(rows, columns=["t_source", "device_serial", "segment",
                                     "px", "py", "pz", "qx", "qy", "qz", "qw", "valid"])
    df = df.sort_values("t_source").reset_index(drop=True)
    out = ROOT / "data" / "mock"; out.mkdir(parents=True, exist_ok=True)
    df.to_csv(out / "mock_session.csv", index=False)

    tt = np.arange(0, DURATION, 0.001); truth = {"t": tt}
    for seg in DEVICES:
        p, q = operator_motion(seg, tt)
        for k, c in enumerate("xyz"): truth[f"{seg}_p{c}"] = p[:, k]
        for k, c in enumerate("xyzw"): truth[f"{seg}_q{c}"] = q[:, k]
    pd.DataFrame(truth).to_csv(out / "mock_truth.csv", index=False)
    (out / "mock_meta.json").write_text(json.dumps(
        {"seed": seed, "yaw0_deg": 37.0, "offset_m": t_offset.tolist(), "rate_hz": 90,
         "dropout": "foot_left 6.00-6.15 s", "invalid": "hand_left 4.00-4.11 s"}, indent=2))
    print(f"wrote {len(df)} samples for {len(DEVICES)} devices -> {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--seed", type=int, default=0)
    main(ap.parse_args().seed)
