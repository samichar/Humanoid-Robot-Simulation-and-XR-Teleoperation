"""Interactive viewer: loads the G1 at the 'stand' keyframe, holds it in real time.
Close the window to exit. Double-click a body + Ctrl+right-drag to push it."""
import time
import mujoco
import mujoco.viewer
from config import SCENE_XML, KEYFRAME

M = mujoco.MjModel.from_xml_path(str(SCENE_XML))
D = mujoco.MjData(M)
mujoco.mj_resetDataKeyframe(M, D, mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_KEY, KEYFRAME))
with mujoco.viewer.launch_passive(M, D) as v:
    while v.is_running():
        t = time.perf_counter()
        mujoco.mj_step(M, D)
        v.sync()
        time.sleep(max(0.0, M.opt.timestep - (time.perf_counter() - t)))
