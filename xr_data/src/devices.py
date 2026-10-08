"""pose sources. Every source implements:
    names            -> list of device names
    poll()           -> {name: (pos_xyz[m], quat_wxyz, valid, confidence)}
confidence = OpenVR ETrackingResult code (200=Running_OK, 201=Running_OutOfRange, 1=Uninitialized...).
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

CONF_OK = 200


def _xyzw_to_wxyz(q):
    return np.array([q[3], q[0], q[1], q[2]])


class MockSource:
    """Fake humanoid-ish motion + realistic defects (jitter, random invalid frames, a disconnect, stalls)."""

    BASE = {"head": (0, 1.65, 0), "left_hand": (-0.25, 1.0, 0.1), "right_hand": (0.25, 1.0, 0.1),
        "pelvis": (0, 0.95, 0), "right_foot": (0.1, 0.1, 0), "left_foot": (-0.1, 0.1, 0)}

    def __init__(self, devices, sequence="neutral", seed=0, disconnect=(None, 0.0)):
        self.devs = devices
        self.names = [d["name"] for d in devices]
        self.seq = sequence
        self.rng = np.random.default_rng(seed)
        self.t0 = time.perf_counter()
        self.disc_start, self.disc_len = disconnect    # seconds into recording; applies to tracker_1

    def poll(self):
        t = time.perf_counter() - self.t0
        if self.rng.random() < 0.005:          # simulated scheduling stall -> dropped ticks
            time.sleep(0.03)
        out = {}
        for d in self.devs:
            p = np.array(self.BASE[d["role"]], dtype=float)
            if self.seq == "arm_raise" and d["role"] in ("left_hand", "right_hand"):
                p[1] += 0.35 * (1 - np.cos(2 * np.pi * 0.25 * t))
            if self.seq == "squat" and d["role"] in ("head", "pelvis"):
                p[1] -= 0.25 * (1 - np.cos(2 * np.pi * 0.2 * t))
            if self.seq == "step" and d["role"] == "right_foot":
                p[1] += max(0.0, 0.15 * np.sin(2 * np.pi * 0.5 * t)); p[2] -= 0.1 * np.sin(2 * np.pi * 0.5 * t)
            p += self.rng.normal(0, 0.0007, 3)                 # ~0.7 mm tracking noise
            rot = R.from_euler("y", 3 * np.sin(0.3 * t) + self.rng.normal(0, 0.1), degrees=True)
            q = _xyzw_to_wxyz(rot.as_quat())
            valid, conf = True, CONF_OK
            if self.rng.random() < 0.01:                       # random invalid frame
                valid, conf = False, 1
            if (d["name"] == "tracker_1" and self.disc_start is not None
                    and self.disc_start <= t < self.disc_start + self.disc_len):
                valid, conf = False, 1                         # disconnect window
            if not valid:
                p = np.full(3, np.nan); q = np.full(4, np.nan)
            out[d["name"]] = (p, q, valid, conf)
        return out


class OpenVRSource:
    """Real hardware through SteamVR (Quest 3 via Link/Air Link/Virtual Desktop + Vive trackers).
    NOT tested without hardware. Devices are matched to names by serial number in devices.json;
    HMD/controllers with an empty serial are matched by device class + role (left/right hand)."""

    def __init__(self, devices):
        import openvr
        self.ov = openvr
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.devs = devices
        self.names = [d["name"] for d in devices]
        self.index = self._map_devices()

    def _serial(self, i):
        return self.vr.getStringTrackedDeviceProperty(i, self.ov.Prop_SerialNumber_String)

    def _map_devices(self):
        ov, index = self.ov, {}
        for i in range(ov.k_unMaxTrackedDeviceCount):
            cls = self.vr.getTrackedDeviceClass(i)
            if cls == ov.TrackedDeviceClass_Invalid:
                continue
            serial = self._serial(i)
            for d in self.devs:
                if d["name"] in index:
                    continue
                if d["serial"] and d["serial"] == serial:
                    index[d["name"]] = i
                elif not d["serial"]:
                    if d["type"] == "hmd" and cls == ov.TrackedDeviceClass_HMD:
                        index[d["name"]] = i
                    elif d["type"] == "controller" and cls == ov.TrackedDeviceClass_Controller:
                        hand = self.vr.getControllerRoleForTrackedDeviceIndex(i)
                        want = ov.TrackedControllerRole_LeftHand if "left" in d["role"] else ov.TrackedControllerRole_RightHand
                        if hand == want:
                            index[d["name"]] = i
        missing = [n for n in self.names if n not in index]
        if missing:
            print(f"[warn] devices not found (will be recorded as invalid): {missing}")
        return index

    def poll(self):
        ov = self.ov
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            ov.TrackingUniverseStanding, 0.0,                  # 0.0 s = pose for 'now', no prediction offset
            ov.k_unMaxTrackedDeviceCount)
        out = {}
        for n in self.names:
            i = self.index.get(n)
            if i is None or not poses[i].bPoseIsValid:
                out[n] = (np.full(3, np.nan), np.full(4, np.nan), False, int(poses[i].eTrackingResult) if i is not None else 0)
                continue
            m = np.array([[poses[i].mDeviceToAbsoluteTracking[r][c] for c in range(4)] for r in range(3)])
            q = _xyzw_to_wxyz(R.from_matrix(m[:, :3]).as_quat())
            out[n] = (m[:, 3].copy(), q, True, int(poses[i].eTrackingResult))
        return out

    def close(self):
        self.ov.shutdown()
