# Data dictionary (schema v1.0)

Each recording = `<id>.npz` (arrays) + `<id>.json` (metadata). Raw files live in `data/raw/`, processed in `data/processed/`.
Raw files are never edited.

## Arrays in the `.npz`
Let N = number of samples, `<dev>` = device name from `configs/devices.json`.

| Key | Shape | Type | Units | Meaning |
|---|---|---|---|---|
| `t_system` | (N,) | float64 | s | Time of the poll, from `time.perf_counter()`, relative to recording start. One shared clock for all devices (software timestamp, not hardware-synced). |
| `<dev>__position` | (N,3) | float64 | m | x, y, z in the tracking frame. NaN when `valid` is False. |
| `<dev>__quat_wxyz` | (N,4) | float64 | unitless | Unit quaternion, order **w,x,y,z**. NaN when invalid. |
| `<dev>__valid` | (N,) | bool | - | Runtime reported the pose as valid (raw), or sample is within `max_gap_s` of real data (processed). |
| `<dev>__confidence` | (N,) | int16 | - | Raw: OpenVR `ETrackingResult` (200 = Running_OK, 1 = Uninitialized, 201 = OutOfRange...). Processed: 200 if valid else 0. |

## Metadata keys in the `.json`
| Key | Meaning |
|---|---|
| `schema_version` | Schema version string |
| `recording_id`, `sequence`, `operator` | Name of the file, motion type (neutral / arm_raise / squat / step), who recorded |
| `source` | `mock` or `openvr` (**mock files are fake test data**) |
| `created_utc` | ISO timestamp |
| `clock` | Description of timestamp source |
| `target_rate_hz` | Requested poll rate (actual rate is in the quality plots) |
| `units`, `orientation`, `coordinate_convention` | Units, quaternion order, axes/handedness |
| `device_table` | name, type, role (body segment), serial for every device |
| `calibration_file`, `calibration_sha256`, `calibration` | Calibration used, its hash, and a full copy of its contents |
| `software` | Python / numpy / OS versions |
| `devices`, `n_samples` | Device names and sample count |
| `processed`, `source_raw`, `processing` | Only in processed files: raw file used, rate, max gap, filter settings |

## Processing rules (raw -> processed)
- **Synchronization:** all devices are polled in the same loop iteration and share one timestamp per tick. Limitation: sub-millisecond offsets between devices and the runtime's own latency are not measured.
- **Resampling:** common 60 Hz grid. Position linear; orientation SLERP. Only valid raw samples are anchors.
- **Missing samples:** an output frame is valid only if the valid raw samples on both sides are <= 0.1 s apart; otherwise it stays NaN / invalid. Nothing is extrapolated outside the first/last valid sample.
- **Filtering:** 2nd-order zero-phase Butterworth low-pass (8 Hz default) per contiguous valid segment. Quaternion components are sign-aligned, filtered, then renormalized. Zero-phase filtering is offline only (non-causal), so it is not usable for the live pipeline.
