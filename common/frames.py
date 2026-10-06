"""Single source of truth for coordinate-frame and quaternion conventions.

SteamVR / OpenVR tracking space: right-handed, +Y up, +X right, -Z forward. Meters.
MuJoCo world (and G1 body frames): right-handed, +Z up, +X forward, +Y left. Meters.

Quaternion orders:
  scipy Rotation ........ (x, y, z, w)   "xyzw"
  MuJoCo qpos / mocap ... (w, x, y, z)   "wxyz"
Recordings store quaternions as xyzw, in SteamVR axes, exactly as captured.

Never convert axes or quaternion order anywhere else in the codebase.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

# Rotation taking SteamVR-axis vectors to MuJoCo-axis vectors:
#   mj_x = -steam_z (forward), mj_y = -steam_x (left), mj_z = steam_y (up)
C_STEAM_TO_MJ = np.array([[0.0, 0.0, -1.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0]])
assert np.isclose(np.linalg.det(C_STEAM_TO_MJ), 1.0)  # proper rotation, no handedness flip


def xyzw_to_wxyz(q):
    q = np.asarray(q)
    return np.concatenate([q[..., 3:4], q[..., 0:3]], axis=-1)


def wxyz_to_xyzw(q):
    q = np.asarray(q)
    return np.concatenate([q[..., 1:4], q[..., 0:1]], axis=-1)


def steam_pos_to_mj(p):
    """(..., 3) positions in SteamVR axes -> MuJoCo axes."""
    return np.asarray(p) @ C_STEAM_TO_MJ.T


def steam_quat_to_mj(q_xyzw):
    """(..., 4) xyzw orientations in SteamVR axes -> xyzw in MuJoCo axes.

    Conjugation re-expresses both the world frame and the device's local frame,
    so a device's local -Z (SteamVR forward) becomes local +X (MuJoCo forward)."""
    Rs = R.from_quat(q_xyzw).as_matrix()
    Rm = C_STEAM_TO_MJ @ Rs @ C_STEAM_TO_MJ.T
    return R.from_matrix(Rm).as_quat()


def mj_pos_to_steam(p):
    return np.asarray(p) @ C_STEAM_TO_MJ


def mj_quat_to_steam(q_xyzw):
    Rm = R.from_quat(q_xyzw).as_matrix()
    return R.from_matrix(C_STEAM_TO_MJ.T @ Rm @ C_STEAM_TO_MJ).as_quat()


if __name__ == "__main__":
    # Self-test: round trips and known directions.
    rng = np.random.default_rng(0)
    p = rng.standard_normal((100, 3)); q = R.random(100, random_state=1).as_quat()
    assert np.allclose(mj_pos_to_steam(steam_pos_to_mj(p)), p)
    assert np.allclose(np.abs(np.sum(mj_quat_to_steam(steam_quat_to_mj(q)) * q, axis=1)), 1.0)
    assert np.allclose(xyzw_to_wxyz(wxyz_to_xyzw(q)), q)
    assert np.allclose(steam_pos_to_mj([0, 1, 0]), [0, 0, 1])    # up stays up
    assert np.allclose(steam_pos_to_mj([0, 0, -1]), [1, 0, 0])   # forward stays forward
    assert np.allclose(steam_pos_to_mj([1, 0, 0]), [0, -1, 0])   # right -> -left
    print("frames.py self-test passed")
