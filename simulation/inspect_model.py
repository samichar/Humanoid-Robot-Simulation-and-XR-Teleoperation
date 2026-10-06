"""Generate docs/robot.md: actuated joints, limits, actuator gains."""
import mujoco
from config import SCENE_XML, ROOT

M = mujoco.MjModel.from_xml_path(str(SCENE_XML))
lines = ["# Unitree G1 (MuJoCo Menagerie): actuated joints", "",
         f"nq={M.nq}, nv={M.nv}, nu={M.nu}, timestep={M.opt.timestep} s, "
         f"total mass={M.body_mass.sum():.2f} kg", "",
         "Units: rad, m, s, N, N*m. World frame is Z-up. "
         "qpos[0:3] = pelvis position, qpos[3:7] = pelvis quaternion (w, x, y, z).",
         "Actuators are position servos: ctrl is a target angle (rad), clipped to ctrlrange.", "",
         "| idx | actuator / joint | qpos adr | lower (rad) | upper (rad) | kp (N*m/rad) | torque limit (N*m) |",
         "|---|---|---|---|---|---|---|"]
for a in range(M.nu):
    j = M.actuator_trnid[a, 0]
    lo, hi = M.jnt_range[j]
    if M.actuator_forcelimited[a]:
        lim = f"{M.actuator_forcerange[a, 1]:.1f}"
    elif M.jnt_actfrclimited[j]:
        lim = f"{M.jnt_actfrcrange[j, 1]:.1f}"
    else:
        lim = "unlimited"
    lines.append(f"| {a} | {mujoco.mj_id2name(M, mujoco.mjtObj.mjOBJ_ACTUATOR, a)} | {M.jnt_qposadr[j]} "
                 f"| {lo:.4f} | {hi:.4f} | {M.actuator_gainprm[a, 0]:.0f} | {lim} |")
(ROOT / "docs").mkdir(exist_ok=True)
(ROOT / "docs" / "robot.md").write_text("\n".join(lines) + "\n")
print("\n".join(lines))
