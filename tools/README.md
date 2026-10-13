# Tools

`mock_xr.py` generates a **synthetic XR recording** in the same format as real SteamVR captures, along with the true motion it was generated from. It tests the simulator-side pipeline (frame conversion → calibration → resampling → replay → retargeting) against a known correct answer, without hardware.

It is **pipeline validation only**, not part of the dataset. For testing the recorder itself, use `xr_data`'s `--source mock` instead.

## Run

From `simulation\` with the venv active:

```bat
python ..\tools\mock_xr.py
python replay.py
python replay.py --view
python retarget.py --view
```

- `mock_xr.py` writes `data\mock\` at the repo root.
- `replay.py` replays and checks against ground truth, writing `results\replay_report.json`.
- `replay.py --view` shows colored markers around the G1.
- `retarget.py --view` makes the robot follow the mock motion.

Run `mock_xr.py` before `replay.py`, or replay fails with `FileNotFoundError`. Use `--seed N` for a different noise realization.

## Mock session

10 s, 6 devices: `head`, `left_hand`, `right_hand`, `pelvis`, `left_foot`, `right_foot`.

| Time | Motion |
|---|---|
| 0–2 s | Neutral stand (used for calibration) |
| 2–5 s | Both arms raise forward |
| 5–8 s | Squat |
| 8–10 s | Step in place: left foot, then right foot |

Injected imperfections:

| Imperfection | Tests |
|---|---|
| Operator 1.2 m off-center and rotated 37° in tracking space | Calibration recovers heading and origin |
| ~90 Hz with ±1 ms timestamp jitter | Resampling irregular timing |
| 1 mm position noise | Realistic sensor noise |
| `right_foot` dropout 6.00–6.15 s (rows missing) | Gap detection |
| `left_hand` flagged `valid=0` 4.00–4.11 s | Invalid-sample handling |

## Files

| File | Contents |
|---|---|
| `data/mock/mock_session.csv` | Recording in SteamVR axes (Y-up), quaternion order xyzw |
| `data/mock/mock_truth.csv` | Ground truth at 1 kHz in the operator frame (MuJoCo axes, Z-up) |
| `data/mock/mock_meta.json` | Seed and injected offsets and faults |

CSV schema: `t_source, device_serial, segment, px, py, pz, qx, qy, qz, qw, valid`.

## Expected results (`replay.py`)

| Check | Result |
|---|---|
| Recovered heading / origin / scale | 37.0°, (1.20, −0.60) m, 0.79 |
| Position error vs. truth | ≈ 1.1 mm RMS per device (the injected noise), max 3.4 mm |
| Orientation error | < 0.1° |
| Invalid steps at 50 Hz | `right_foot` 8, `left_hand` 6 (gaps flagged, not interpolated) |
| `truth_check_passed` | `true` (threshold: max position error < 10 mm) |

During invalid steps, targets hold their last valid pose.

## Limitations

- The mock does not model drift, occlusion, or tracker slip; real recordings must still pass the `xr_data` quality checks.
