# Simulation

MuJoCo 3.15 simulation of the Unitree G1 humanoid, plus the simulator side of the XR pipeline: replaying recorded motion and retargeting it onto the robot.

| Script | Purpose | Output |
|---|---|---|
| `config.py` | Shared paths, model, keyframe, control rate | |
| `view.py` | Interactive viewer, G1 holding its `stand` keyframe | |
| `inspect_model.py` | Joint inventory: actuators, `qpos` indices, limits, gains, torque limits | `docs/robot.md` |
| `smoke_tests.py` | Task 1 verification suite | `results/smoke_tests.json` |
| `replay.py` | Recording → calibration → 50 Hz → target markers around the G1 | `results/replay_report.json`, `replay_log.npz` |
| `retarget.py` | Whole-body IK: tracked targets → G1 joint angles | `results/retarget_<id>.json`, `retarget_<id>.npz` |

Setup commands run from the repo root; all other commands run from `simulation\` with the venv active.

## Setup

These steps were tested on Windows with 64-bit Python 3.13.

### 1. Prerequisites

You need two things installed before starting.

**Git for Windows**: https://git-scm.com. Check that it works:

```bat
git --version
```

**Python 3.13 (64-bit)**: https://www.python.org/downloads/windows/. Use the **"Windows installer (64-bit)"**. During install, keep **"py launcher"** checked.

MuJoCo only ships prebuilt wheels for specific Python versions, so the version matters:

| Python | Works? | Why |
|---|---|---|
| 3.6 and other versions older than 3.10 | No | Too old. `pip install mujoco` fails with `No matching distribution found` |
| 3.13 (64-bit, python.org) | **Yes, use this** | Tested |
| 3.14 | Not recommended | Too new; dependencies may not have wheels yet |
| Microsoft Store builds (e.g. "3.11 (Store)") | Not recommended | Sandboxed; causes path and permission problems |
| Any 32-bit Python or ARM64 build | No | No MuJoCo wheels |

### 2. Check which Pythons are installed

Windows machines often have several Pythons installed, and plain `python` or `python3` may point at the wrong one. List them with the py launcher:

```bat
py -0
```

Example output:

```
 -V:3.14[-64] *   Python 3.14.5
 -V:3.13          Python 3.13 (64-bit)
 -V:3.11          Python 3.11 (Store)
 -V:3.6           Python 3.6 (64-bit)
```

Confirm `3.13 (64-bit)` is listed. **Always create the venv with `py -3.13`**, never with bare `python`, because bare `python` may resolve to an old interpreter on your PATH. On our machine it resolved to 3.6.

To see exactly what an interpreter is:

```bat
python -c "import sys, platform; print(sys.version); print(platform.architecture()[0], platform.machine())"
```

### 3. Clone the repo (outside OneDrive)

**Do not put the project inside OneDrive, Dropbox, or any synced folder.** OneDrive locks files while syncing. This caused `Unable to copy ... venvlauncher.exe` errors when creating the venv, and it can corrupt git repos. GitHub is our sync mechanism.

```bat
mkdir C:\dev
cd C:\dev
git clone <repo-url> Humanoid-Robot-Simulation-and-XR-Teleoperation
cd Humanoid-Robot-Simulation-and-XR-Teleoperation
git submodule update --init
```

`git submodule update --init` downloads the robot model (see section 4). Without it, `third_party\mujoco_menagerie` will be empty.

### 4. Robot model: MuJoCo Menagerie submodule

The Unitree G1 model comes from Google DeepMind's [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie). It is included as a **git submodule** pinned to a specific commit, so every teammate loads the identical model.

| Item | Value |
|---|---|
| Path | `third_party\mujoco_menagerie` |
| Pinned commit | `f054586a8e90465d49ee5be15335c4a0c7f57caf` |
| Scene loaded | `third_party\mujoco_menagerie\unitree_g1\scene.xml` |
| License | BSD-3-Clause (Unitree Robotics); see `unitree_g1\LICENSE` |

For reference, this is how the submodule was originally added (**already done; teammates do not need to run this**):

```bat
git submodule add https://github.com/google-deepmind/mujoco_menagerie third_party/mujoco_menagerie
cd third_party\mujoco_menagerie
git checkout f054586a8e90465d49ee5be15335c4a0c7f57caf
cd ..\..
git add .
git commit -m "Add MuJoCo Menagerie (G1) as pinned submodule"
```

Verify the model is present:

```bat
dir third_party\mujoco_menagerie\unitree_g1\scene.xml
```

### 5. Create the virtual environment (once per machine)

The venv lives at the **repo root** as `.venv` and is excluded from git by `.gitignore`. Create it **once** from the repo root:

```bat
py -3.13 -m venv .venv
```

Notes:

- Run this from the repo root, not from `C:\Users\<you>`. Otherwise the venv ends up in your user folder.
- If you **move or rename** the project folder, delete and recreate the venv. A venv stores absolute paths and breaks when moved:
  ```bat
  rmdir /s /q .venv
  py -3.13 -m venv .venv
  ```
- If venv creation fails with `Unable to copy ... python.exe`, the old venv is in use. Close VS Code and all terminals using it, check that the folder is not inside OneDrive, then retry.

### 6. Activate the venv (every new terminal)

You create the venv once, but you must **activate it in every new terminal window**:

```bat
.venv\Scripts\activate
```

Your prompt should now start with `(.venv)`. Confirm:

```bat
python --version
where python
```

`python --version` must print `Python 3.13.x`, and the first line of `where python` must point inside `...\.venv\Scripts\`.

**If you see `ModuleNotFoundError: No module named 'mujoco'`, the venv is not active.** Your prompt will be missing `(.venv)`. Activate it and rerun.

From inside the `simulation` folder, activate with:

```bat
..\.venv\Scripts\activate
```

To skip activation, call the venv's Python directly:

```bat
..\.venv\Scripts\python.exe smoke_tests.py
```

**VS Code:** press `Ctrl+Shift+P`, choose **Python: Select Interpreter**, and pick `.venv`. New VS Code terminals will then activate it automatically.

### 7. Install dependencies

With the venv active:

```bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt -r xr_data\requirements.txt
```

Use `python -m pip` rather than bare `pip`, and never `python3` on Windows. `python -m pip` guarantees packages install into the interpreter you are actually running. On Windows, `python3` is usually a Microsoft Store alias, not your venv.

Check the installed MuJoCo version and make sure it matches the pin in `requirements.txt`:

```bat
python -m pip show mujoco
```

## Verify the simulator

All scripts live in `simulation\`:

```bat
cd simulation
python view.py
python inspect_model.py
python smoke_tests.py
```

| Script | What it does | Expected result |
|---|---|---|
| `view.py` | Opens the interactive viewer with the G1 holding its `stand` keyframe | Window opens and the robot stands. Double-click a body, then Ctrl + right-drag to push it. |
| `inspect_model.py` | Writes `docs\robot.md`: all 29 actuated joints, `qpos` indices, joint limits, gains, torque limits | Joint table printed and file written |
| `smoke_tests.py` | Runs the Task 1 verification suite and writes `results\smoke_tests.json` | Ends with `ALL PASSED` |

Smoke tests covered:

| Test | Checks |
|---|---|
| `model_load` | Model loads: 29 actuators, one floating base |
| `conventions` | Gravity (0, 0, -9.81), SI units, 0.002 s timestep, 10 physics substeps per 50 Hz control step |
| `stand_5s` | Position servos hold the `stand` keyframe for 5 s |
| `actuator_step` | Left elbow tracks a +0.5 rad target step |
| `ctrl_clamp` | Out-of-range commands are clipped to the joint limit |
| `reset_determinism` | Identical reset and command sequence produce bitwise-identical state |
| `contact_forces` | Summed foot normal force equals body weight while standing |
| `push_*` | Informational: response to pelvis shoves. The robot has no balance controller yet and falls. |
| `state_logging` | State logged at 50 Hz, written to disk, reread identically |

## Model and conventions

| Item | Value |
|---|---|
| Robot | Unitree G1, 29 actuated joints (no hands), MuJoCo Menagerie `unitree_g1/scene.xml` @ `f054586` |
| State sizes | `nq` = 36 (7 floating base + 29 joints), `nv` = 35, `nu` = 29 |
| Mass | 33.34 kg |
| Actuators | Position servos, kp = 500 N·m/rad, critically damped; `ctrl` = target joint angle (rad), clipped to `ctrlrange` = joint range |
| `qpos` layout | `[0:3]` pelvis position, `[3:7]` pelvis quaternion (w, x, y, z), `[7:36]` joints in actuator order |
| Joint order | 0–5 left leg, 6–11 right leg, 12–14 waist, 15–21 left arm, 22–28 right arm (full table in `docs/robot.md`) |
| World frame | Z-up, gravity (0, 0, −9.81), +X forward, +Y left |
| Rates | Physics 500 Hz (`timestep` 0.002 s); control and logging 50 Hz (10 substeps per control step) |

## Replay: recorded motion into the simulator

```bat
python replay.py
python replay.py --view
python replay.py --recording ..\xr_data\data\raw\arm_wave_raise.npz --view
```

- The first command replays the mock session headless and checks it against ground truth.
- `--view` shows the same replay in the viewer.
- `--recording` replays a real recording from `xr_data`.

Pipeline:
1. Load the recording. `.npz` (xr_data format) and `.csv` (mock format) are both accepted.
2. Convert frames with `common/frames.py`.
3. Calibrate: heading, origin, floor, scale.
4. Resample to 50 Hz: linear interpolation for position, slerp for orientation. Gaps longer than 50 ms are flagged invalid, not interpolated.
5. Drive one colored mocap marker per tracked segment. During invalid samples, a marker holds its last valid pose.

**Calibration is currently a placeholder.** Heading comes from the headset's forward direction, origin from the mean pelvis position, and scale from robot pelvis height ÷ operator pelvis height. All three are estimated from the first 1.5 s of each recording, so **every recording must begin with 2–3 s of standing still in the neutral pose.** Task 2's calibration file will supply these values instead.

## Retargeting: making the robot follow the person

```bat
python retarget.py --view
python retarget.py --recording ..\xr_data\data\raw\arm_wave_raise.npz --view
python retarget.py --mode physics --view
python retarget.py --no-filter
```

- The first two commands use kinematic mode: the robot is posed directly from IK.
- `--mode physics` sends the joint angles to the servos with no balance controller, so the robot falls.
- `--no-filter` disables filtering and the velocity clamp, for comparison.

**Method.** Each 50 Hz frame solves damped-least-squares inverse kinematics over all 35 degrees of freedom (floating base + 29 joints), warm-started from the previous frame.

| Target | Robot site | Matched | Weight |
|---|---|---|---|
| `pelvis` | pelvis | position + orientation | 10 |
| `left_foot`, `right_foot` | ankle roll links, 3 cm below | position + orientation | 10 |
| `left_hand`, `right_hand` | wrist yaw links, 8 cm forward | position | 3 |
| `head` | torso, 45 cm up | position | 0.5 |

**Neutral-pose offsets.** During the neutral window, a constant position and rotation offset is stored per segment so the operator's neutral pose maps exactly onto the robot's stand pose. This absorbs limb-length and tracker-mounting differences, which is why tracker mounting angle does not matter as long as the tracker is rigid.

**Stability measures:**

| Setting | Value | Purpose |
|---|---|---|
| Median filter + 4 Hz zero-phase Butterworth | `FILTER_HZ = 4.0` | Remove tracker spikes and jitter |
| Quaternion sign continuity | | `q` and `−q` are the same rotation; prevents filter artifacts |
| Soft-knee neutral pose | `SOFT_KNEE = 0.25` rad | A straight knee sits on its joint limit and is singular; bent knees prevent leg flips |
| IK step cap + iterations | `MAX_IK_STEP = 0.05` rad/iteration, `ITERS = 30` | Prevents jumps to a different solution branch and back-and-forth chatter when a target is at the edge of reach |
| Damping | `DAMPING = 0.05` | Steadier solutions near singularities |
| Temporal smoothness | `SMOOTH_W = 0.05` | Pull toward the previous frame's pose |
| Posture regularization | `POSTURE_W = 0.05` | Pull unconstrained joints toward the stand pose |
| Joint-limit clamp | per joint, from the model | Joint-limit handling |
| Joint velocity clamp (output) | `MAX_JOINT_SPEED = 10` rad/s | Command clamping (safety layer) |

**Results:**

| Recording | Max joint speed | Mean IK error (pelvis / feet / right hand) | Joint-limit hits |
|---|---|---|---|
| Mock, noisy, before stability fixes | 264 rad/s | leg flips, foot error ~1 m | |
| Mock, noisy (`--no-filter`) | 17.8 rad/s | 0.5 / 0.3 / 1.3 mm | 15 |
| Mock, noisy (filtered) | 7.2 rad/s | 0.2 / 0.1 / 0.7 mm | 8 |
| Real: `neutral_final` | 0.8 rad/s | 0.7 / 0.0 / 3.2 mm | 0 |
| Real: `arm_wave_raise` | 3.6 rad/s | 1.4 / 0.4 / 13.1 mm | 0 |
| Real: `leg_raise` | 3.2 rad/s | 3.5 / 3.4 / 12.0 mm | 703 |

Retargeting a 30 s recording takes about 20 s offline (about 77 frames/s), faster than the 50 Hz control rate.

Hand errors on real data reflect the G1's shorter arm reach. The `leg_raise` limit hits come from hip motion beyond the G1's joint range; joints are clamped at their limits.

**Kinematic vs. physics.** Kinematic mode is a puppet: it follows the reference with no notion of balance. In physics mode the same joint angles go to the servos and the robot falls (pelvis ≈ 0.06 m). Tracking the reference while staying balanced is the Milestone 2 policy's job.

## Known constraints

- The keyframe stand is a stiff PD posture with no balance control; the robot falls under a 0.5 m/s pelvis shove.
- Retargeting filters offline (zero-phase); live teleoperation needs a causal filter, which adds lag.
- Calibration is a placeholder until Task 2 delivers `calibration.json` values.
- GPU training (MJX / MuJoCo Playground) needs JAX with CUDA: Linux or WSL2, not native Windows.
- Python 3.6 cannot install MuJoCo; always create the venv with `py -3.13`. Do not clone into OneDrive.

## Troubleshooting (errors we hit)


| Error | Cause | Fix |
|---|---|---|
| `ERROR: No matching distribution found for mujoco` | Python is too old (3.6), too new, 32-bit, or ARM | Use 64-bit Python 3.13; create the venv with `py -3.13` |
| Same error, right Python | Typo in package name | It's `mujoco` |
| `ModuleNotFoundError: No module named 'mujoco'` | venv not active, so `python` resolved to system Python | `.venv\Scripts\activate`, then check for `(.venv)` in the prompt |
| `'..\.venv\Scripts\activate' is not recognized` | venv was created in a different folder (e.g. `C:\Users\<you>\.venv`) | Create it at the repo root with `py -3.13 -m venv .venv` |
| `Unable to copy ... venvlauncher.exe ... python.exe` | Existing venv in use, or OneDrive locking files | Close VS Code and terminals; move the project out of OneDrive; delete `.venv` and recreate |
| Error loading `scene.xml` / file not found | Submodule not downloaded or folder misnamed | `git submodule update --init`; path must be `third_party\mujoco_menagerie\unitree_g1\scene.xml` |
| `ModuleNotFoundError: No module named 'matplotlib'` | Only the root `requirements.txt` was installed | `python -m pip install -r xr_data\requirements.txt` (or merge the two files) |
