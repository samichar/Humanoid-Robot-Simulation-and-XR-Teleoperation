"""fixed-rate recorder. one shared clock (time.perf_counter) stamps every tick

ctrl c = stop and save

TEST:  python src/record.py --source mock --sequence arm_raise --duration 20 --out data/raw/mock_arm_raise_01.npz
REAL:  python src/record.py --source openvr --sequence neutral --duration 30 --out data/raw/neutral_01.npz
"""
import argparse, json, time, platform, sys, datetime
import numpy as np
from devices import MockSource, OpenVRSource
from io_utils import save_recording, sha256_of


def record(source, names, rate_hz, duration_s):
    period = 1.0 / rate_hz
    t_list = []
    buf = {n: {"position": [], "quat_wxyz": [], "valid": [], "confidence": []} for n in names}
    t0 = time.perf_counter()
    k = 0
    try:
        while True:
            target = t0 + k * period
            wait = target - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            t = time.perf_counter() - t0
            if t >= duration_s:
                break
            sample = source.poll()
            t_list.append(t)
            for n in names:
                p, q, v, c = sample[n]
                buf[n]["position"].append(p); buf[n]["quat_wxyz"].append(q)
                buf[n]["valid"].append(v); buf[n]["confidence"].append(c)
            # if we fell behind, skip missed ticks
            k = max(k + 1, int((time.perf_counter() - t0) / period) + 1)
    except KeyboardInterrupt:
        print("\n[stop] interrupted, saving what was recorded")
    data = {n: {"position": np.array(b["position"], float), "quat_wxyz": np.array(b["quat_wxyz"], float),
                "valid": np.array(b["valid"], bool), "confidence": np.array(b["confidence"], np.int16)}
            for n, b in buf.items()}
    return np.array(t_list), data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["mock", "openvr"], default="mock")
    ap.add_argument("--devices", default="configs/devices.json")
    ap.add_argument("--calib", default="configs/calibration.json")
    ap.add_argument("--sequence", required=True, help="neutral | arm_raise | squat | step ...")
    ap.add_argument("--operator", default="unknown")
    ap.add_argument("--rate", type=float, default=90.0, help="target poll rate (Hz)")
    ap.add_argument("--duration", type=float, default=30.0, help="seconds")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=0, help="mock only")
    ap.add_argument("--mock-disconnect", type=float, nargs=2, metavar=("START_S", "LEN_S"), default=None)
    a = ap.parse_args()

    devices = json.load(open(a.devices))["devices"]
    names = [d["name"] for d in devices]
    if a.source == "mock":
        src = MockSource(devices, a.sequence, a.seed, tuple(a.mock_disconnect) if a.mock_disconnect else (None, 0.0))
    else:
        src = OpenVRSource(devices)

    print(f"Recording '{a.sequence}' for {a.duration}s at {a.rate} Hz  (Ctrl+C to stop early)")
    t, data = record(src, names, a.rate, a.duration)
    if hasattr(src, "close"):
        src.close()

    meta = {
        "recording_id": a.out.split("/")[-1].rsplit(".", 1)[0],
        "sequence": a.sequence, "operator": a.operator, "source": a.source,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "clock": "time.perf_counter, seconds since recording start (software timestamp at poll time)",
        "target_rate_hz": a.rate,
        "units": {"position": "meters", "time": "seconds"},
        "orientation": "unit quaternion, order w,x,y,z, rotation of device frame in tracking frame",
        "coordinate_convention": "OpenVR standing universe: right-handed, +x right, +y up, -z forward, origin at floor",
        "device_table": devices,
        "calibration_file": a.calib, "calibration_sha256": sha256_of(a.calib),
        "calibration": json.load(open(a.calib)) if sha256_of(a.calib) != "none" else None,
        "software": {"python": sys.version.split()[0], "numpy": np.__version__, "os": platform.platform()},
    }
    save_recording(a.out, t, data, meta)
    print(f"Saved {len(t)} samples -> {a.out.rsplit('.', 1)[0]}.npz/.json")


if __name__ == "__main__":
    main()
