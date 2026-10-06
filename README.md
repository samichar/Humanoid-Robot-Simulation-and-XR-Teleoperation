## Simulation Environment Setup (Milestone 1, Task 1)

These steps set up the MuJoCo simulator and the Unitree G1 humanoid on Windows and verify that the environment works. They were tested on Windows with 64-bit Python 3.13.

### 1. Prerequisites

You need two things installed before starting.

**Git for Windows**: https://git-scm.com. Check that it works:

```bat
git --version
```

**Python 3.13 (64-bit)**: https://www.python.org/downloads/windows/. Use the **"Windows installer (64-bit)"**. During install, keep **"py launcher"** checked.

MuJoCo only ships prebuilt wheels for specific Python versions, so the version matters:

| Python                                       | Works?            | Why                                                                       |
| -------------------------------------------- | ----------------- | ------------------------------------------------------------------------- |
| 3.6 and other versions older than 3.10       | No                | Too old. `pip install mujoco` fails with `No matching distribution found` |
| 3.13 (64-bit, python.org)                    | **Yes, use this** | Tested                                                                    |
| 3.14                                         | Not recommended   | Too new; dependencies may not have wheels yet                             |
| Microsoft Store builds (e.g. "3.11 (Store)") | Not recommended   | Sandboxed; causes path and permission problems                            |
| Any 32-bit Python or ARM64 build             | No                | No MuJoCo wheels                                                          |

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

| Item          | Value                                                     |
| ------------- | --------------------------------------------------------- |
| Path          | `third_party\mujoco_menagerie`                            |
| Pinned commit | `f054586a8e90465d49ee5be15335c4a0c7f57caf`                |
| Scene loaded  | `third_party\mujoco_menagerie\unitree_g1\scene.xml`       |
| License       | BSD-3-Clause (Unitree Robotics); see `unitree_g1\LICENSE` |

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
python -m pip install -r requirements.txt
```

Use `python -m pip` rather than bare `pip`, and never `python3` on Windows. `python -m pip` guarantees packages install into the interpreter you are actually running. On Windows, `python3` is usually a Microsoft Store alias, not your venv.

Check the installed MuJoCo version and make sure it matches the pin in `requirements.txt`:

```bat
python -m pip show mujoco
```

### 8. Run and verify the simulator

All scripts live in `simulation\`:

```bat
cd simulation
python view.py
python inspect_model.py
python smoke_tests.py
```

| Script             | What it does                                                                                       | Expected result                                                                            |
| ------------------ | -------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `view.py`          | Opens the interactive viewer with the G1 holding its `stand` keyframe                              | Window opens and the robot stands. Double-click a body, then Ctrl + right-drag to push it. |
| `inspect_model.py` | Writes `docs\robot.md`: all 29 actuated joints, `qpos` indices, joint limits, gains, torque limits | Joint table printed and file written                                                       |
| `smoke_tests.py`   | Runs the Task 1 verification suite and writes `results\smoke_tests.json`                           | Ends with `ALL PASSED`                                                                     |

Smoke tests covered:

| Test                | Checks                                                                                        |
| ------------------- | --------------------------------------------------------------------------------------------- |
| `model_load`        | Model loads: 29 actuators, one floating base                                                  |
| `conventions`       | Gravity (0, 0, -9.81), SI units, 0.002 s timestep, 10 physics substeps per 50 Hz control step |
| `stand_5s`          | Position servos hold the `stand` keyframe for 5 s                                             |
| `actuator_step`     | Left elbow tracks a +0.5 rad target step                                                      |
| `ctrl_clamp`        | Out-of-range commands are clipped to the joint limit                                          |
| `reset_determinism` | Identical reset and command sequence produce bitwise-identical state                          |
| `contact_forces`    | Summed foot normal force equals body weight while standing                                    |
| `push_*`            | Informational: response to pelvis shoves. The robot has no balance controller yet and falls.  |
| `state_logging`     | State logged at 50 Hz, written to disk, reread identically                                    |

### 9. Quick start (after first-time setup)

Every time you open a new terminal:

```bat
cd C:\dev\Humanoid-Robot-Simulation-and-XR-Teleoperation
.venv\Scripts\activate
cd simulation
python smoke_tests.py
```

### 10. Troubleshooting (errors we hit)

| Error                                                | Cause                                                                | Fix                                                                                             |
| ---------------------------------------------------- | -------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `ERROR: No matching distribution found for mujoco`   | Python is too old (3.6), too new, 32-bit, or ARM                     | Use 64-bit Python 3.13; create the venv with `py -3.13`                                         |
| Same error, right Python                             | Typo in package name                                                 | It's `mujoco`                                                                                   |
| `ModuleNotFoundError: No module named 'mujoco'`      | venv not active, so `python` resolved to system Python               | `.venv\Scripts\activate`, then check for `(.venv)` in the prompt                                |
| `'..\.venv\Scripts\activate' is not recognized`      | venv was created in a different folder (e.g. `C:\Users\<you>\.venv`) | Create it at the repo root with `py -3.13 -m venv .venv`                                        |
| `Unable to copy ... venvlauncher.exe ... python.exe` | Existing venv in use, or OneDrive locking files                      | Close VS Code and terminals; move the project out of OneDrive; delete `.venv` and recreate      |
| Error loading `scene.xml` / file not found           | Submodule not downloaded or folder misnamed                          | `git submodule update --init`; path must be `third_party\mujoco_menagerie\unitree_g1\scene.xml` |
