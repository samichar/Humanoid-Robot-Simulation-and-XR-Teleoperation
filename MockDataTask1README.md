## Mock XR Data: Pipeline Testing Without Hardware

`tools/mock_xr.py` generates a **synthetic XR recording** in the same format real SteamVR captures will use. It lets us test the simulator-side pipeline (frame conversion → calibration → resampling → replay in MuJoCo) before the Quest 3 and Vive tracker setup (Task 2) is ready.

It is **pipeline validation only**. It does not replace the real Task 3 dataset.

### What the mock session contains

A 10 s session for 5 devices: headset (`head`), two controllers (`hand_left`, `hand_right`), and two Vive trackers (`pelvis`, `foot_left`).

| Time   | Motion                               |
| ------ | ------------------------------------ |
| 0–2 s  | Neutral stand (used for calibration) |
| 2–5 s  | Both arms raise forward              |
| 5–8 s  | Squat                                |
| 8–10 s | Left foot lifts                      |

Realistic imperfections are injected on purpose:

| Imperfection                                                | Purpose                                     |
| ----------------------------------------------------------- | ------------------------------------------- |
| Operator 1.2 m off-center and rotated 37° in tracking space | Calibration must recover heading and origin |
| ~90 Hz with ±1 ms timestamp jitter                          | Resampler must handle irregular timing      |
| 1 mm position noise                                         | Realistic sensor noise                      |
| `foot_left` dropout, 6.00–6.15 s (rows missing)             | Gap detection                               |
| `hand_left` flagged `valid=0`, 4.00–4.11 s                  | Invalid-sample handling                     |

### Files

| File                         | Contents                                                               |
| ---------------------------- | ---------------------------------------------------------------------- |
| `data/mock/mock_session.csv` | The recording, in SteamVR axes (Y-up), quaternion order xyzw           |
| `data/mock/mock_truth.csv`   | Ground-truth motion at 1 kHz in the operator frame (MuJoCo axes, Z-up) |
| `data/mock/mock_meta.json`   | Seed and injected offsets/faults                                       |

Recording schema (real recordings must use the same columns):

```
t_source, device_serial, segment, px, py, pz, qx, qy, qz, qw, valid
```

### Run

From the repo root, with the venv active:

```bat
python common\frames.py
python tools\mock_xr.py
cd simulation
python replay.py
python replay.py --view
```

- `frames.py` self-tests the axis and quaternion conversions.
- `mock_xr.py` writes the mock data to `data\mock\`. Run it before `replay.py`, or replay fails with `FileNotFoundError`.
- `replay.py` replays headless, checks against ground truth, and writes `results\replay_report.json`.
- `replay.py --view` replays in the viewer: colored spheres move around the G1.

Use `--seed N` with `mock_xr.py` to generate a different noise realization.

### Expected results

| Check                              | Result                                                        |
| ---------------------------------- | ------------------------------------------------------------- |
| Recovered heading / origin / scale | 37.0°, (1.20, −0.60) m, 0.79                                  |
| Position error vs. truth           | ≈1.1 mm RMS per device (the injected noise), max < 4 mm       |
| Orientation error                  | < 0.1°                                                        |
| Invalid steps at 50 Hz             | `foot_left` 8, `hand_left` 6 (gaps flagged, not interpolated) |
| `truth_check_passed`               | `true` (pass threshold: max position error < 10 mm)           |

During invalid steps, targets hold their last valid pose.

### Limitations

- Calibration in `replay.py` is a **placeholder** that estimates heading, origin, floor, and scale from the first 1.5 s. Task 2's calibration replaces it with the same four values.
- The mock does not model real tracking failures such as drift, occlusion, or tracker slip. Real recordings must still pass the Task 3 quality checks.
- To replay a real recording: `python replay.py --recording ..\data\<file>.csv` (no truth check).
