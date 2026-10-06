# Task 3: Synchronized data acquisition (Person C)

## Setup
    python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
    pip install -r requirements.txt
    pip freeze > requirements.lock.txt                      # commit this

## Test without hardware (mock devices)
    python src/record.py --source mock --sequence arm_raise --duration 15 --mock-disconnect 5 1.0 --out data/raw/mock_arm_raise_01.npz
    python src/process.py data/raw/mock_arm_raise_01.npz data/processed/mock_arm_raise_01_60hz.npz
    python src/plot_quality.py data/raw/mock_arm_raise_01.npz --outdir plots

## With real hardware (SteamVR running, Quest 3 connected, trackers paired and green)
    python src/list_devices.py                  # copy tracker serials into configs/devices.json
    python src/record.py --source openvr --sequence neutral   --duration 30 --operator NAME --out data/raw/neutral_01.npz
    python src/record.py --source openvr --sequence arm_raise --duration 45 --operator NAME --out data/raw/arm_raise_01.npz
    python src/record.py --source openvr --sequence squat     --duration 45 --operator NAME --out data/raw/squat_01.npz
    python src/process.py data/raw/neutral_01.npz data/processed/neutral_01_60hz.npz
    python src/plot_quality.py data/raw/neutral_01.npz --outdir plots

Ctrl+C stops a recording early and still saves it.

## Files
- `src/devices.py`    mock + OpenVR pose sources (OpenVR path untested until hardware is available)
- `src/record.py`     fixed-rate recorder, one shared clock
- `src/process.py`    resample (linear / SLERP), low-pass, gap handling
- `src/plot_quality.py` rate, gaps, ranges, trajectories + `*_quality.json`
- `src/io_utils.py`   all file reading/writing
- `docs/data_dictionary.md`  schema and processing rules
- `configs/devices.json`, `configs/calibration.json` (calibration is a placeholder until Person A delivers)

## Known limitations
- Software timestamps at poll time (no hardware sync); real latency not measured.
- Mock data is for testing only and must not be submitted as the dataset.
