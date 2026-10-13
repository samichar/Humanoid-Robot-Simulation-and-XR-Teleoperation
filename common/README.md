# Common

`frames.py` is the **only** place in the codebase that converts between coordinate frames or quaternion orders. Every other module imports from it.

| Frame | Axes | Units |
|---|---|---|
| SteamVR / OpenVR tracking space | Right-handed, +Y up, +X right, −Z forward | m |
| MuJoCo world and G1 body frames | Right-handed, +Z up, +X forward, +Y left | m |

| Library | Quaternion order |
|---|---|
| MuJoCo (`qpos`, `mocap_quat`) | (w, x, y, z) |
| SciPy `Rotation` | (x, y, z, w) |
| `xr_data` recordings (`.npz`) | (w, x, y, z) |
| Mock recordings (`.csv`) | (x, y, z, w) |

## Axis mapping

SteamVR → MuJoCo is a proper rotation (determinant +1, no handedness flip):

| MuJoCo | = SteamVR |
|---|---|
| x (forward) | −z |
| y (left) | −x |
| z (up) | y |

Orientations are converted by conjugation, R_mj = C R_steam Cᵀ, which re-expresses both the world frame and the device's own local frame. A device's local −Z (SteamVR forward) therefore becomes local +X (MuJoCo forward).

## Functions

| Function | Converts |
|---|---|
| `steam_pos_to_mj(p)` / `mj_pos_to_steam(p)` | (…, 3) positions |
| `steam_quat_to_mj(q)` / `mj_quat_to_steam(q)` | (…, 4) xyzw orientations |
| `xyzw_to_wxyz(q)` / `wxyz_to_xyzw(q)` | Quaternion order |

## Self-test

```bat
python common\frames.py
```

Checks round trips and known directions: up stays up, forward stays forward, right maps to −left. It should print `frames.py self-test passed`.
