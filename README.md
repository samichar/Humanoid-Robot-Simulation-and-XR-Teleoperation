# Humanoid Robot Simulation and Extended Reality (XR) Teleoperation

ELEG 4870V/5870V Introduction to Robotics, University of Arkansas (Cognitive Robotics Lab).

## Team

| Member               | Role                      | Technical contribution                                                            |
| -------------------- | ------------------------- | --------------------------------------------------------------------------------- |
| Samantha Jackson     | Simulation lead (Task 1)  | MuJoCo + G1 environment, frame conventions, replay/retargeting into the simulator |
| Mary-Claire Ridgeway | XR hardware lead (Task 2) | Meta Quest 3 + Vive setup, device mapping, calibration, repeatability             |
| Katelyn Adair        | Data lead (Task 3)        | Recorder, processing, quality plots, data dictionary, documentation               |
| Lillian Morris       |                           |                                                                                   |

## System Overview

```
Quest 3 + controllers + 3 Vive trackers
        │  SteamVR / OpenVR (pyopenvr)
        ▼
xr_data/   record.py ──► data/raw/<id>.npz + .json      (raw, never modified)
           process.py ─► data/processed/<id>_50hz.npz   (resampled, filtered)
           plot_quality.py ─► plots/                    (quality diagnostics)
        │
        ▼
common/frames.py   SteamVR (Y-up, xyzw) ──► MuJoCo (Z-up, wxyz)
        │
        ▼
simulation/
   replay.py    calibrate, resample to 50 Hz, show tracked targets around the G1
   retarget.py  whole-body IK: targets ──► 29 G1 joint angles
        │
        ▼
MuJoCo 3.15 + Unitree G1 (MuJoCo Menagerie)
```

## Repository layout

| Folder                          | Contents                                                       | README                                       |
| ------------------------------- | -------------------------------------------------------------- | -------------------------------------------- |
| `simulation/`                   | MuJoCo environment, smoke tests, replay, retargeting           | [simulation/README.md](simulation/README.md) |
| `xr_data/`                      | XR capture, processing, quality plots, recorded dataset        | [xr_data/README.md](xr_data/README.md)       |
| `tools/`                        | Mock XR data generator with ground truth                       | [tools/README.md](tools/README.md)           |
| `common/`                       | Shared frame and quaternion conversions                        | [common/README.md](common/README.md)         |
| `data/mock/`                    | Generated mock session (from `tools/mock_xr.py`)               | [tools/README.md](tools/README.md)           |
| `third_party/mujoco_menagerie/` | Unitree G1 model (git submodule, pinned)                       | [simulation/README.md](simulation/README.md) |
| `results/`                      | Script outputs (smoke tests, replay, retargeting); git-ignored |                                              |

## Setup (Windows)

Full details and troubleshooting are in [simulation/README.md](simulation/README.md#setup).

Prerequisites:

- Git for Windows
- **64-bit Python 3.13** from python.org
- For real hardware: SteamVR, the Quest 3 connected through Steam Link or Quest Link, and the Vive trackers paired

```bat
cd C:\dev
git clone https://github.com/samichar/Humanoid-Robot-Simulation-and-XR-Teleoperation.git
cd Humanoid-Robot-Simulation-and-XR-Teleoperation
git submodule update --init
py -3.13 -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt -r xr_data\requirements.txt
```

Clone outside OneDrive or any other synced folder. Create the venv once; **activate it in every new terminal** (`.venv\Scripts\activate`). The prompt should show `(.venv)`.

## Quick start

```bat
cd simulation
python smoke_tests.py
python view.py
python ..\tools\mock_xr.py
python replay.py --view
python retarget.py --view
python retarget.py --recording ..\xr_data\data\raw\arm_wave_raise.npz --view
```

- `smoke_tests.py` verifies the simulator and should end with `ALL PASSED`.
- `view.py` shows the G1 standing.
- `mock_xr.py` generates mock XR data with known ground truth.
- `replay.py --view` shows the tracked targets around the robot.
- `retarget.py --view` makes the robot follow the mock motion.
- The last command does the same with a real recording.

## Full pipeline: record → process → retarget

Records real motion and plays it on the G1. Data flows Task 2 (hardware) → Task 3 (data) → Task 1 (simulation).

Pick one recording ID and use it in every command. Here it is `arm_raise_01`.

**1. Record (from `xr_data\`)**

Before starting: SteamVR running, Quest 3 connected, all three trackers green.

```bat
cd xr_data
python src\list_devices.py
python src\record.py --source openvr --sequence arm_raise --duration 30 --operator SAM --out data/raw/arm_raise_01.npz
python src\process.py data/raw/arm_raise_01.npz data/processed/arm_raise_01_50hz.npz --rate 50
python src\plot_quality.py data/raw/arm_raise_01.npz --outdir plots
```

- `list_devices.py` confirms all six devices are visible.
- Start every recording with 2–3 s of standing still in the neutral pose.
- `process.py` resamples and filters the recording.
- `plot_quality.py` writes the quality plots; check them before moving on.

**2. Retarget onto the robot (from `simulation\`)**

```bat
cd ..\simulation
python replay.py --recording ..\xr_data\data\raw\arm_raise_01.npz --view
python retarget.py --recording ..\xr_data\data\raw\arm_raise_01.npz --view
```

- `replay.py` is optional: it shows the tracked targets as dots.
- `retarget.py` makes the robot follow the motion.

| Step | Script | Output |
|---|---|---|
| Device check | `xr_data/src/list_devices.py` | Device list with serials |
| Record | `xr_data/src/record.py` | `xr_data/data/raw/<id>.npz` + `.json` |
| Process | `xr_data/src/process.py` | `xr_data/data/processed/<id>_50hz.npz` |
| Quality check | `xr_data/src/plot_quality.py` | `xr_data/plots/<id>_*.png`, `<id>_quality.json` |
| Replay (optional) | `simulation/replay.py` | `results/replay_report.json` |
| Retarget | `simulation/retarget.py` | `results/retarget_<id>.npz` + `.json` |

Retargeting reads the **raw** file because it does its own 50 Hz resampling and filtering. The processed file is for quality checks and the dataset submission.

## Citations and licenses

| Component                                                                                        | Use                                      | License                         |
| ------------------------------------------------------------------------------------------------ | ---------------------------------------- | ------------------------------- |
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.15.0                                       | Physics simulator                        | Apache-2.0                      |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) `unitree_g1` @ `f054586` | G1 robot model                           | BSD-3-Clause (Unitree Robotics) |
| [pyopenvr](https://github.com/cmbruns/pyopenvr) / SteamVR                                        | XR pose access                           | BSD-3-Clause                    |
| NumPy, SciPy, pandas, Matplotlib                                                                 | Numerics, rotations, filtering, plotting | BSD / PSF-compatible            |
