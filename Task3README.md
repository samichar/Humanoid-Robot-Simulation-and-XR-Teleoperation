Task 3: Synchronized Data Acquisition

Course: ELEG 4870V / 5870V Introduction to Robotics, Humanoid Robot Simulation and XR Teleoperation Milestone 1 (Midterm), Task 3 (12 points) Author: Person C (data pipeline)  |  Team: [add names and each member's technical contribution]

1. What this is and why it exists

The project goal is to make a simulated humanoid robot copy human motion. The robot can only learn from human motion if we first have a recording of a human moving, and that recording must be trustworthy. Task 3 is that recording step:

Acquisition: record the Meta Quest 3 headset/controllers and at least two Vive trackers for at least three motions (neutral, upper body, lower body).
Data pipeline: store the data in a documented machine-readable format, and explain synchronization, resampling, filtering, and missing-sample handling.
Quality checks: plot sample rates, gaps, pose ranges, and synchronized trajectories; replay a recording through the simulator input pipeline; list remaining data limitations.

This folder contains the code and documentation for all three parts. It is the data foundation that the final-project training (Milestone 2) will use.

2. Current status (read this first)
Item	Status
Recorder, processing, plotting, file format, data dictionary	Written and tested on mock (fake) data
Real hardware path (--source openvr)	Written but NOT yet tested. No headset or trackers were available when this was built. Expect small fixes on first real run, most likely in device matching.
Calibration file	Placeholder (identity transform). To be replaced by the Task 2 Quest-to-Vive calibration.
Real dataset (3 sequences)	Not yet recorded
Replay into simulator	Not yet done. Depends on Task 1 / simulator code.
Limitations list	To be written from real data plots

Everything under data/ right now is mock data for testing only. Do not submit it as the dataset.

3. How this was built (history)
Read the project PDF and extracted the Task 3 requirements (acquisition, data pipeline, quality checks).
Because the hardware and the simulator were still being set up by teammates, the pipeline was built against a mock device that generates fake poses for head, hands, pelvis, and foot, with deliberately added defects (tracking noise, random invalid frames, scheduling stalls that drop ticks, and an optional tracker disconnect). This allowed every downstream step to be developed and tested without hardware.
Designed the file schema first, then the recorder, then the processing, then the plots, then the data dictionary.
Ran the whole chain end to end on a 15-second mock recording with a 1-second injected tracker disconnect.
Wrote the real SteamVR/OpenVR source from the pyopenvr API so that switching to hardware is one flag (--source openvr).
Result of the mock test (not real data)
Metric	Value
Samples recorded	1,334 over 15.0 s
Mean sample rate	88.9 Hz (target 90 Hz)
Interval std. dev. / max	1.8 ms / 37.0 ms
Dropped ticks detected	17
Invalid frames	about 1% per device, 7.6% on tracker_1 (it contained the injected 1 s disconnect)

The injected disconnect appeared as a block of invalid frames in the gap plot, and the processing step left it unfilled, which is the intended behavior.

4. How it works
 Quest 3 + Vive trackers
        |   (SteamVR / OpenVR)
        v
  devices.py  -- poll() returns pose + valid flag for every device
        |
        v
  record.py   -- loop at fixed rate, stamps every tick with ONE shared clock
        |        saves RAW file (never modified afterward)
        v
  data/raw/<id>.npz + .json
        |
        v
  process.py  -- resample to 60 Hz, SLERP, low-pass, leave long gaps empty
        |
        v
  data/processed/<id>_60hz.npz + .json
        |
        +--> plot_quality.py -> plots/*.png + *_quality.json
        +--> replay (simulator side, Person B)
4.1 src/devices.py (where poses come from)

Every source offers names and poll(). poll() returns, for each device, (position_xyz_m, quaternion_wxyz, valid, confidence).

MockSource: fake motion for the sequences neutral, arm_raise, squat, step, plus noise, random invalid frames, stalls, and an optional disconnect window on tracker_1.
OpenVRSource: reads real poses from SteamVR. It requests the pose for "now" (secondsFromNow = 0) so timestamps are not shifted by prediction. Devices are matched to names by serial number (trackers) or by device class and hand (HMD, controllers). Rotation matrices from OpenVR are converted to quaternions in w,x,y,z order. Devices that cannot be found are recorded as invalid and a warning is printed.
4.2 src/record.py (the recorder)
Polls every device in one loop at a fixed target rate (default 90 Hz).
Every tick is stamped with time.perf_counter(), a monotonic clock, relative to the recording start. All devices share that one timestamp per tick.
If the loop falls behind, it skips the missed ticks instead of squeezing samples together, so dropped frames show up honestly as gaps in the timestamps.
Ctrl+C stops early and still saves. This also works as a manual stop for the operator.
Writes metadata with every recording: sequence name, operator, device table (name, role, serial), units, coordinate convention, software versions, and the calibration file's name, SHA-256 hash, and full contents. The hash proves which calibration produced a recording.
4.3 src/process.py (raw to processed)

The raw file is never modified. Processing writes a new file.

Synchronization: already handled at record time by the shared per-tick timestamp. Limitation: this is a software timestamp, not hardware sync.
Resampling: onto a uniform 60 Hz grid. Position uses linear interpolation. Orientation uses SLERP (spherical interpolation), because quaternions must not be interpolated component by component.
Missing samples: only valid raw samples are used as anchors. An output frame is marked valid only if the valid raw samples on both sides of it are at most 0.1 s apart. Longer gaps stay NaN and invalid. Nothing is extrapolated before the first or after the last valid sample.
Filtering: 2nd-order zero-phase Butterworth low-pass (default 8 Hz), applied separately to each continuous valid segment so it never smooths across a gap. Quaternions are sign-aligned, filtered, and renormalized. Zero-phase filtering is non-causal, so it is for offline data only and is not suitable for the live pipeline.
4.4 src/plot_quality.py (quality checks)

Produces, per recording, with units on every axis:

<id>_rate.png: histogram of intervals between samples and sample rate versus time
<id>_gaps.png: timeline of invalid frames per device and dropped ticks
<id>_ranges.png: min and max of x, y, z per device (to check physical plausibility)
<id>_trajectories.png: all devices' x, y, z versus time on shared axes
<id>_quality.json: the same numbers in machine-readable form (mean rate, interval statistics, dropped ticks, invalid percentage and position ranges per device)
4.5 src/io_utils.py

All reading and writing is in this one module. Each recording is <id>.npz (arrays) plus <id>.json (metadata). HDF5 was the first choice, but it could not be installed in the build environment, so .npz plus JSON was used (no extra dependency, fully machine-readable). Switching to HDF5 later only requires changing this file.

4.6 docs/data_dictionary.md

Every array and metadata field with type, units, and meaning, plus the processing rules. This is what Person B needs in order to read the files correctly.

5. Setup
bash
python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pip freeze > requirements.lock.txt   # commit this (the PDF requires a lock file)

Tested with Python 3, numpy 2.4.4, scipy 1.17.1, matplotlib 3.10.8. The openvr package is only needed for real hardware.

6. Run it
6.1 Without hardware (mock data)
bash
python src/record.py --source mock --sequence arm_raise --duration 15 \
    --mock-disconnect 5 1.0 --out data/raw/mock_arm_raise_01.npz
python src/process.py data/raw/mock_arm_raise_01.npz data/processed/mock_arm_raise_01_60hz.npz
python src/plot_quality.py data/raw/mock_arm_raise_01.npz --outdir plots
6.2 With real hardware

Before starting: SteamVR is running, the Quest 3 is connected, trackers are paired and show green, guardian boundary is enabled, wrist straps are on, and an emergency stop is available to the operator and an observer (see the PDF safety section).

bash
python src/list_devices.py     # prints index, class, serial of every device
# copy the tracker serials into configs/devices.json (match each to its body segment)
# put the Task 2 calibration into configs/calibration.json

python src/record.py --source openvr --sequence neutral   --duration 30 --operator NAME --out data/raw/neutral_01.npz
python src/record.py --source openvr --sequence arm_raise --duration 45 --operator NAME --out data/raw/arm_raise_01.npz
python src/record.py --source openvr --sequence squat     --duration 45 --operator NAME --out data/raw/squat_01.npz

python src/process.py data/raw/neutral_01.npz data/processed/neutral_01_60hz.npz
python src/plot_quality.py data/raw/neutral_01.npz --outdir plots

Do a 10 second test recording and check the plots before the real session. Record several takes per sequence.

6.3 Command options

record.py: --source mock|openvr, --sequence, --duration (s), --rate (Hz, default 90), --operator, --devices, --calib, --out. process.py: --rate (default 60), --max-gap (s, default 0.1), --cutoff (Hz, default 8, use 0 to disable). plot_quality.py: --outdir, --gap-factor (default 1.5).

7. Loading the data in Python (for Person B)
python
import numpy as np, json
z = np.load("data/processed/neutral_01_60hz.npz")
meta = json.load(open("data/processed/neutral_01_60hz.json"))
t    = z["t_system"]                 # (N,)   seconds
pos  = z["ctrl_right__position"]     # (N,3)  meters, NaN where invalid
quat = z["ctrl_right__quat_wxyz"]    # (N,4)  order w,x,y,z
ok   = z["ctrl_right__valid"]        # (N,)   skip or hold frames where False

Device names: hmd, ctrl_left, ctrl_right, tracker_1, tracker_2 (see configs/devices.json). Replay code must handle valid == False frames, since their positions are NaN.

8. Repository layout
src/            devices.py  record.py  process.py  plot_quality.py  io_utils.py  list_devices.py
configs/        devices.json (device names, roles, serials)   calibration.json (placeholder)
docs/           data_dictionary.md
data/raw/       raw recordings (never edited)
data/processed/ resampled and filtered recordings
plots/          quality plots and *_quality.json
requirements.txt
9. Known limitations
Timing: timestamps are taken in software when the poll returns, not hardware-synchronized. Runtime latency and sub-millisecond offsets between devices are not measured.
Rate: the achieved rate was below the 90 Hz target in the mock test (88.9 Hz) because of scheduling stalls. Real rates and jitter will depend on the machine and SteamVR.
Two tracking systems: the Quest 3 (inside-out) and Vive trackers (lighthouse) have separate coordinate frames. Data quality depends on the Task 2 calibration; the current file is a placeholder.
Filtering: the 8 Hz zero-phase low-pass is offline only and slightly smooths fast motion. It cannot be used in the live control loop as is.
Gaps: gaps longer than 0.1 s are left empty. Downstream code must handle them.
Real-hardware limitations (drift, dropouts, jitter, tracker slippage) to be added after analyzing the real recordings.
10. Safety and data rules (from the PDF)
Use the headset only in a cleared play area with the guardian enabled and wrist straps secured. Stop immediately for discomfort, loss of balance, or tracking instability.
Record only ordinary system-performance motion logs. Do not collect biometric or research-participant data without instructor and institutional approval.
Do not commit credentials or headset pairing secrets.
11. Remaining to-do
 Fill in tracker serials and body mapping once hardware is connected
 Replace placeholder calibration with the Task 2 result
 Test and fix the OpenVR path on real hardware
 Record neutral, upper-body, and lower-body sequences (several takes each)
 Process the real recordings and generate quality plots
 Replay at least one recording with Person B's simulator code
 Write the real limitations list
 Remove or clearly label the mock files before submission
 Pin versions in requirements.lock.txt
 Add team member names and contributions
12. Citations

numpy, scipy (signal, spatial.transform), matplotlib, and pyopenvr (OpenVR / SteamVR). [add versions, licenses, and any tutorials or third-party code used]