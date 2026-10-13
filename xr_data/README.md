# XR Data Acquisition

Captures Meta Quest 3 headset and controller poses plus three Vive trackers through SteamVR, stores them in a documented machine-readable format, resamples and filters them, and produces quality diagnostics. This is the data foundation for replay, retargeting, and Milestone 2 training.

All commands run from `xr_data\` with the venv active. Setup is in the [main README](../README.md#setup-windows).

## Pipeline

```
Quest 3 + controllers + Vive trackers
        │  SteamVR / OpenVR
        ▼
src/devices.py   poll() → position, quaternion (wxyz), valid, confidence per device
        ▼
src/record.py    fixed-rate loop (90 Hz), one shared clock per tick
        ▼
data/raw/<id>.npz + <id>.json          (never modified)
        ▼
src/process.py   resample to 50 Hz, SLERP, low-pass, leave long gaps empty
        ▼
data/processed/<id>_50hz.npz + .json
        ├─► src/plot_quality.py → plots/*.png + *_quality.json
        └─► ../simulation/replay.py, retarget.py
```

| File | Role |
|---|---|
| `src/devices.py` | `MockSource` (fake motion with injected defects) and `OpenVRSource` (real SteamVR poses) |
| `src/record.py` | Fixed-rate recorder; Ctrl+C stops early and still saves |
| `src/process.py` | Raw → processed: resampling, gap handling, filtering |
| `src/plot_quality.py` | Rate, gap, range, and trajectory plots plus machine-readable summary |
| `src/io_utils.py` | All file reading and writing |
| `src/list_devices.py` | Prints index, class, and serial of every connected device |
| `configs/devices.json` | Device name → body segment → serial |
| `configs/calibration.json` | Quest ↔ Vive alignment and floor height (placeholder) |
| `docs/data_dictionary.md` | Every array and metadata field, with units and processing rules |

## Devices

| Name | Type | Segment | Serial |
|---|---|---|---|
| `hmd` | Quest 3 headset | `head` | matched by device class |
| `ctrl_left` | Quest 3 controller | `left_hand` | matched by controller role |
| `ctrl_right` | Quest 3 controller | `right_hand` | matched by controller role |
| `tracker_1` | Vive tracker | `pelvis` | `59-A33R04534` |
| `tracker_2` | Vive tracker | `right_foot` | `59-A33R00919` |
| `tracker_3` | Vive tracker | `left_foot` | `59-A33R04591` |

Trackers are matched **only** by serial; SteamVR device indices change between sessions. Label each physical tracker to match this table.

**Recommended mounting** (record the actual placement with photos):
- **Pelvis:** center of the lower back on a tight belt.
- **Feet:** top of each foot over the laces, strapped tight.

Mounting angle does not matter, because retargeting absorbs it during the neutral stand. What matters is that each tracker is rigid and mounted the same way every session. If a tracker is re-strapped, re-record the neutral stand.

## Recording procedure

Before starting:
- SteamVR is running and the Quest 3 is connected.
- All three trackers show green in SteamVR.
- The guardian boundary is enabled and wrist straps are on.
- An emergency stop is available to both operator and observer.

1. Check that every device is visible:
   ```bat
   python src\list_devices.py
   ```
2. Record each sequence. **Start every recording with 2–3 s of standing still in the neutral pose**; the simulator calibrates from it.
   ```bat
   python src\record.py --source openvr --sequence neutral   --duration 30 --operator NAME --out data/raw/neutral_01.npz
   python src\record.py --source openvr --sequence arm_raise --duration 45 --operator NAME --out data/raw/arm_raise_01.npz
   python src\record.py --source openvr --sequence squat     --duration 45 --operator NAME --out data/raw/squat_01.npz
   python src\record.py --source openvr --sequence step      --duration 45 --operator NAME --out data/raw/step_01.npz
   ```
   Always set `--sequence` and `--operator`; they are stored in the metadata.
3. Process and check each recording:
   ```bat
   python src\process.py data/raw/arm_raise_01.npz data/processed/arm_raise_01_50hz.npz
   python src\plot_quality.py data/raw/arm_raise_01.npz --outdir plots
   ```

If `record.py` prints `[warn] devices not found`, stop and restart. Devices are matched once at startup, so a device that was off at that moment is recorded as invalid for the whole file.

Without hardware, use `--source mock` (optionally `--mock-disconnect 5 1.0` to inject a 1 s tracker dropout).

### Command options

| Script | Options |
|---|---|
| `record.py` | `--source mock\|openvr`, `--sequence`, `--duration` (s), `--rate` (Hz, default 90), `--operator`, `--devices`, `--calib`, `--out` |
| `process.py` | `--rate` (Hz, default 50), `--max-gap` (s, default 0.1), `--cutoff` (Hz, default 8; 0 disables) |
| `plot_quality.py` | `--outdir`, `--gap-factor` (default 1.5) |

## Data format

Each recording is `<id>.npz` (arrays) plus `<id>.json` (metadata). Full schema: [docs/data_dictionary.md](docs/data_dictionary.md).

| Array | Shape | Units | Meaning |
|---|---|---|---|
| `t_system` | (N,) | s | Shared timestamp per tick (`time.perf_counter()`, relative to start) |
| `<dev>__position` | (N, 3) | m | Position in SteamVR tracking space (Y-up). NaN when invalid |
| `<dev>__quat_wxyz` | (N, 4) | | Orientation, order **w, x, y, z**. NaN when invalid |
| `<dev>__valid` | (N,) | bool | Pose valid (raw) or within the max gap of real data (processed) |
| `<dev>__confidence` | (N,) | | Raw: OpenVR `ETrackingResult` (200 = Running_OK) |

Metadata includes the sequence, operator, source (`mock` or `openvr`), device table with serials, coordinate convention, software versions, and the calibration file's name, SHA-256 hash, and full contents.

```python
import numpy as np, json
z = np.load("data/processed/arm_raise_01_50hz.npz")
meta = json.load(open("data/processed/arm_raise_01_50hz.json"))
t, pos, ok = z["t_system"], z["tracker_1__position"], z["tracker_1__valid"]
```

## Processing rules

| Step | Rule |
|---|---|
| Synchronization | All devices are polled in one loop iteration and share one software timestamp per tick |
| Dropped ticks | If the loop falls behind, missed ticks are skipped, so dropped frames appear as timestamp gaps |
| Resampling | Uniform 50 Hz grid (matches the simulator control rate). Position linear, orientation SLERP; only valid samples are anchors |
| Missing samples | A frame is valid only if the valid samples on both sides are ≤ 0.1 s apart; longer gaps stay NaN. No extrapolation |
| Filtering | 2nd-order zero-phase Butterworth, 8 Hz, per contiguous valid segment; quaternions sign-aligned and renormalized. Offline only |

## Quality checks

`plot_quality.py` writes, per recording:

| File | Contents |
|---|---|
| `<id>_rate.png` | Interval histogram and sample rate over time |
| `<id>_gaps.png` | Invalid frames per device and dropped ticks |
| `<id>_ranges.png` | Min/max x, y, z per device (physical plausibility) |
| `<id>_trajectories.png` | All devices' x, y, z over time |
| `<id>_quality.json` | Mean rate, interval statistics, dropped ticks, invalid %, ranges |
