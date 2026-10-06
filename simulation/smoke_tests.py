"""Task 1 simulator verification: model, conventions, standing, actuator, clamping,
reset determinism, contact, push response, state logging.

Usage:  python sim/smoke_tests.py      -> prints PASS/FAIL, writes results/smoke_tests.json
Exit code is nonzero if any test fails.
"""
import json, platform, sys, time
import numpy as np
import mujoco
from config import SCENE_XML, KEYFRAME, CONTROL_HZ, RESULTS, MENAGERIE_COMMIT

M = mujoco.MjModel.from_xml_path(str(SCENE_XML))
KEY = mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_KEY, KEYFRAME)
RESULTS.mkdir(exist_ok=True)
results = {}


def record(name, passed, **info):
    results[name] = {"passed": bool(passed), **info}
    print(f"[{'PASS' if passed else 'FAIL'}] {name}  {info}")


def fresh():
    d = mujoco.MjData(M)
    mujoco.mj_resetDataKeyframe(M, d, KEY)
    return d


def run(d, seconds):
    for _ in range(int(round(seconds / M.opt.timestep))):
        mujoco.mj_step(M, d)


def name(obj, i):
    return mujoco.mj_id2name(M, obj, i)


# 1. Model / joint inventory
free = [j for j in range(M.njnt) if M.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
act_joints = [name(mujoco.mjtObj.mjOBJ_JOINT, M.actuator_trnid[a, 0]) for a in range(M.nu)]
record("model_load", M.nu == 29 and len(free) == 1,
       nq=M.nq, nv=M.nv, nu=M.nu, floating_base=name(mujoco.mjtObj.mjOBJ_JOINT, free[0]))

# 2. Units / conventions: Z-up gravity, SI units, timestep divides control period
dt = float(M.opt.timestep)
decim = (1.0 / CONTROL_HZ) / dt
record("conventions",
       np.allclose(M.opt.gravity, [0, 0, -9.81]) and abs(decim - round(decim)) < 1e-9,
       gravity=M.opt.gravity.tolist(), timestep_s=dt, physics_hz=1 / dt,
       control_hz=CONTROL_HZ, substeps_per_control=int(round(decim)),
       total_mass_kg=float(M.body_mass.sum()), quat_order="wxyz")

# 3. Standing: position actuators hold the keyframe for 5 s
d = fresh(); z0 = float(d.qpos[2]); run(d, 5.0)
record("stand_5s", abs(d.qpos[2] - z0) < 0.02, pelvis_z_start=z0, pelvis_z_end=float(d.qpos[2]))

# 4. Actuator: step the left elbow target by 0.5 rad, check tracking after 1 s
d = fresh(); a = act_joints.index("left_elbow_joint")
qadr = M.jnt_qposadr[M.actuator_trnid[a, 0]]
target = float(d.ctrl[a] + 0.5)
d.ctrl[a] = target; run(d, 1.0)
err = abs(d.qpos[qadr] - target)
record("actuator_step", err < 0.02, joint="left_elbow_joint", target_rad=target, error_rad=float(err))

# 5. Ctrl clamping: an out-of-range command is clipped to ctrlrange
d = fresh(); d.ctrl[a] = 100.0; run(d, 1.0)
hi = float(M.actuator_ctrlrange[a, 1])
record("ctrl_clamp", d.qpos[qadr] <= hi + 0.05, commanded_rad=100.0, ctrl_upper_rad=hi,
       reached_rad=float(d.qpos[qadr]))

# 6. Reset determinism: same reset + same command sequence -> bitwise identical state
def rollout():
    d = fresh(); rng = np.random.default_rng(0)
    for _ in range(100):
        d.ctrl[:] = M.key_ctrl[KEY] + 0.05 * rng.standard_normal(M.nu)
        run(d, 1.0 / CONTROL_HZ)
    return d.qpos.copy()

q1, q2 = rollout(), rollout()
record("reset_determinism", np.array_equal(q1, q2), max_abs_diff=float(np.abs(q1 - q2).max()))

# 7. Contact: summed normal force while standing equals body weight
d = fresh(); run(d, 2.0); f6 = np.zeros(6); normal = 0.0; bodies = set()
for i in range(d.ncon):
    c = d.contact[i]; mujoco.mj_contactForce(M, d, i, f6); normal += f6[0]
    for g in (c.geom1, c.geom2):
        b = M.geom_bodyid[g]
        if b != 0:
            bodies.add(name(mujoco.mjtObj.mjOBJ_BODY, b))
weight = float(M.body_mass.sum() * 9.81)
record("contact_forces", abs(normal - weight) / weight < 0.02,
       n_contacts=int(d.ncon), sum_normal_N=normal, weight_N=weight, bodies=sorted(bodies))

# 8. Push response (informational): shove pelvis, see whether the PD stand survives
for v in (0.5, 1.0, 2.0):
    d = fresh(); d.qvel[0] = v; run(d, 3.0)
    record(f"push_{v}mps", True, pelvis_z_after_s=float(d.qpos[2]), fell=bool(d.qpos[2] < 0.5),
           note="informational; use to pick a fall threshold")

# 9. State logging at CONTROL_HZ: write, reread, verify rate and contents
d = fresh(); rows = []; sub = int(round(decim)); t0 = time.perf_counter()
for _ in range(CONTROL_HZ * 2):
    for _ in range(sub):
        mujoco.mj_step(M, d)
    rows.append(np.concatenate([[d.time], d.qpos, d.qvel, d.ctrl]))
wall = time.perf_counter() - t0
arr = np.array(rows); path = RESULTS / "smoke_state_log.npz"
cols = (["time"] + [f"qpos{i}" for i in range(M.nq)]
        + [f"qvel{i}" for i in range(M.nv)] + [f"ctrl_{n}" for n in act_joints])
np.savez(path, data=arr, columns=np.array(cols))
back = np.load(path)["data"]
rate = (len(back) - 1) / (back[-1, 0] - back[0, 0])
record("state_logging", np.array_equal(arr, back) and abs(rate - CONTROL_HZ) < 1e-6,
       rows=len(back), cols=back.shape[1], logged_hz=float(rate), realtime_factor=float(2.0 / wall))

meta = {"python": platform.python_version(), "os": platform.platform(),
        "mujoco": mujoco.__version__, "numpy": np.__version__,
        "menagerie_commit": MENAGERIE_COMMIT, "model": "unitree_g1/scene.xml"}
(RESULTS / "smoke_tests.json").write_text(json.dumps({"meta": meta, "tests": results}, indent=2))
failed = [k for k, v in results.items() if not v["passed"]]
print("\nALL PASSED" if not failed else f"\nFAILED: {failed}")
sys.exit(1 if failed else 0)
