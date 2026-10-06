"""raw -> processed: resample to a common rate, SLERP quaternions, low-pass filter.
raw files are never modified.

python src/process.py data/raw/x.npz data/processed/x_60hz.npz --rate 60 --max-gap 0.1 --cutoff 8
Rules:
 *only valid raw samples are used as interpolation anchors
 *output sample is valid only if the nearest raw valid samples on BOTH sides are <= max_gap apart
 *low-pass (zero-phase Butterworth) runs per contiguous valid segment; NaN is never smoothed across
"""
import argparse
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R, Slerp
from io_utils import load_recording, save_recording

CONF_OK = 200


def _segments(valid):
    idx = np.flatnonzero(np.diff(np.r_[0, valid.astype(int), 0]))
    return list(zip(idx[::2], idx[1::2]))


def resample_device(t_raw, d, t_new, max_gap):
    v = d["valid"].astype(bool) & np.isfinite(d["position"]).all(1)
    n_new = len(t_new)
    pos = np.full((n_new, 3), np.nan); quat = np.full((n_new, 4), np.nan); ok = np.zeros(n_new, bool)
    if v.sum() < 2:
        return pos, quat, ok
    tv = t_raw[v]
    inside = (t_new >= tv[0]) & (t_new <= tv[-1])
    j = np.searchsorted(tv, t_new[inside])
    j = np.clip(j, 1, len(tv) - 1)
    gap = tv[j] - tv[j - 1]
    good = gap <= max_gap
    tn = t_new[inside]
    for k in range(3):
        pos[inside, k] = np.interp(tn, tv, d["position"][v][:, k])
    q_xyzw = d["quat_wxyz"][v][:, [1, 2, 3, 0]]
    sl = Slerp(tv, R.from_quat(q_xyzw))
    qi = sl(tn).as_quat()[:, [3, 0, 1, 2]]
    quat[inside] = qi
    ok_in = good
    ok[inside] = ok_in
    pos[~ok] = np.nan; quat[~ok] = np.nan
    return pos, quat, ok


def lowpass(pos, quat, ok, rate, cutoff):
    b, a = butter(2, cutoff / (rate / 2))
    pad = 3 * max(len(a), len(b))
    for s, e in _segments(ok):
        if e - s <= pad:
            continue
        pos[s:e] = filtfilt(b, a, pos[s:e], axis=0)
        q = quat[s:e].copy()
        for i in range(1, len(q)):                      # fix sign flips so q and -q don't get averaged
            if np.dot(q[i], q[i - 1]) < 0:
                q[i] = -q[i]
        q = filtfilt(b, a, q, axis=0)
        quat[s:e] = q / np.linalg.norm(q, axis=1, keepdims=True)
    return pos, quat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out")
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--max-gap", type=float, default=0.1, help="s; longer gaps stay missing")
    ap.add_argument("--cutoff", type=float, default=8.0, help="Hz; 0 disables filtering")
    a = ap.parse_args()

    t, data, meta = load_recording(a.inp)
    t_new = np.arange(t[0], t[-1], 1.0 / a.rate)
    out = {}
    for name, d in data.items():
        pos, quat, ok = resample_device(t, d, t_new, a.max_gap)
        if a.cutoff > 0:
            pos, quat = lowpass(pos, quat, ok, a.rate, a.cutoff)
        out[name] = {"position": pos, "quat_wxyz": quat, "valid": ok,
                     "confidence": np.where(ok, CONF_OK, 0).astype(np.int16)}
        print(f"{name:12s} valid {ok.mean()*100:5.1f}% of resampled frames")
    meta = dict(meta, processed=True, source_raw=a.inp,
                processing={"rate_hz": a.rate, "max_gap_s": a.max_gap, "lowpass_cutoff_hz": a.cutoff,
                            "filter": "2nd-order Butterworth, zero-phase (filtfilt), per valid segment",
                            "position_interp": "linear", "orientation_interp": "SLERP"})
    save_recording(a.out, t_new, out, meta)
    print("Saved", a.out)


if __name__ == "__main__":
    main()
