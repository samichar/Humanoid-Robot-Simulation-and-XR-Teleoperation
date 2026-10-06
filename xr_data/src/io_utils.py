"""All file I/O lives here. Format: <name>.npz (arrays) + <name>.json (metadata).
Array keys:  t_system  and  <device>__position / __quat_wxyz / __valid / __confidence
"""
import json, hashlib, os
import numpy as np

SCHEMA_VERSION = "1.0"
FIELDS = ("position", "quat_wxyz", "valid", "confidence")


def sha256_of(path):
    if not path or not os.path.exists(path):
        return "none"
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def save_recording(path, t, data, meta):
    """t: (N,) seconds. data: {device: {position(N,3), quat_wxyz(N,4), valid(N,), confidence(N,)}}"""
    arrays = {"t_system": np.asarray(t, dtype=np.float64)}
    for dev, d in data.items():
        for f in FIELDS:
            arrays[f"{dev}__{f}"] = np.asarray(d[f])
    base = os.path.splitext(path)[0]
    np.savez_compressed(base + ".npz", **arrays)
    meta = dict(meta, schema_version=SCHEMA_VERSION, devices=list(data.keys()), n_samples=int(len(t)))
    with open(base + ".json", "w") as fh:
        json.dump(meta, fh, indent=2)


def load_recording(path):
    base = os.path.splitext(path)[0]
    z = np.load(base + ".npz")
    meta = json.load(open(base + ".json"))
    data = {dev: {f: z[f"{dev}__{f}"] for f in FIELDS} for dev in meta["devices"]}
    return z["t_system"], data, meta
