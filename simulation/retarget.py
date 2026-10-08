r"""Retarget tracked human motion onto the G1 with whole-body inverse kinematics.

Pipeline: recording -> replay.py loading/calibration/resampling (50 Hz targets)
       -> neutral-pose offsets (human neutral == robot 'stand' keyframe)
       -> damped-least-squares IK over all 35 DoF (floating base + 29 joints)
       -> joint-limit clamping -> qpos trajectory

Targets (task -> robot frame, what is matched, weight):
  pelvis      -> pelvis body          position + orientation   high
  left/right_foot -> foot sites       position + orientation   high
  left/right_hand -> wrist link sites position only            medium
  head        -> point above torso    position only            low (leans the torso)

Modes:
  --mode kinematic  (default) robot is posed directly from IK; no physics, no balance.
                     This is the retargeted REFERENCE motion.
  --mode physics    IK joint angles are sent to the position servos and physics runs.
                     Shows what happens with no balance controller (Milestone 2's job).

Usage:
  python retarget.py --view                                        # mock CSV
  python retarget.py --recording ..\data\raw\arm_raise_01.npz --view
Outputs: results/retarget_<id>.npz (t, qpos, joint targets), results/retarget_<id>.json (errors)
"""
import argparse, json, time
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

import replay as rp
from config import SCENE_XML, KEYFRAME, CONTROL_HZ, RESULTS, ROOT

# segment: (robot body, local site offset m, match orientation?, weight)
TASKS = {
    "pelvis":     ("pelvis",                [0, 0, 0],      True,  10.0),
    "left_foot":  ("left_ankle_roll_link",  [0, 0, -0.03],  True,  10.0),
    "right_foot": ("right_ankle_roll_link", [0, 0, -0.03],  True,  10.0),
    "left_hand":  ("left_wrist_yaw_link",   [0.08, 0, 0],   False, 3.0),
    "right_hand": ("right_wrist_yaw_link",  [0.08, 0, 0],   False, 3.0),
    "head":       ("torso_link",            [0, 0, 0.45],   False, 0.5),
}
ROT_W = 0.3          # orientation error weight relative to position (rad vs m)
POSTURE_W = 0.05     # pull joints toward the stand pose when targets don't constrain them
SMOOTH_W = 0.05       # pull joints toward the previous frame's solution (temporal smoothness)
DAMPING = 0.02       # DLS damping (higher = steadier near singularities, e.g. straight knees)
ITERS = 15
MAX_IK_STEP = 0.15   # rad, cap on any joint's change per IK iteration (prevents branch flips)
SOFT_KNEE = 0.25     # rad, neutral knee bend; a straight knee sits on its limit and is singular
FILTER_HZ = 4.0      # low-pass cutoff on tracker targets (human motion is mostly < 4 Hz)
MAX_JOINT_SPEED = 10  # rad/s, joint velocity clamp on the retargeted reference


def build_model(segments):
    spec = mujoco.MjSpec.from_file(str(SCENE_XML))
    for seg in segments:
        if seg in TASKS:
            body, off, _, _ = TASKS[seg]
            spec.body(body).add_site(name=f"ik_{seg}", pos=off, size=[0.02, 0, 0])
        b = spec.worldbody.add_body(name=f"target_{seg}", mocap=True)
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], contype=0,
                   conaffinity=0, rgba=rp.COLORS.get(seg, [1, 1, 1, 1]))
    return spec.compile()


def smooth_targets(rs, hz):
    """Clean tracker targets before IK: spike rejection, quaternion sign continuity,
    zero-phase low-pass. Offline (filtfilt); a live version needs a causal filter."""
    from scipy.signal import butter, filtfilt, medfilt
    b, a = butter(2, FILTER_HZ / (hz / 2))
    for seg, d in rs.items():
        if seg == "t":
            continue
        p = np.stack([medfilt(d["p"][:, k], 5) for k in range(3)], 1)       # kill single-frame spikes
        d["p"] = filtfilt(b, a, p, axis=0)
        q = d["q"].copy()
        for k in range(1, len(q)):                                          # q and -q are the same
            if np.dot(q[k], q[k - 1]) < 0:                                  # rotation; keep the path
                q[k] = -q[k]                                                # continuous before filtering
        q = filtfilt(b, a, q, axis=0)
        d["q"] = q / np.linalg.norm(q, axis=1, keepdims=True)


def site_pose(M, D, sid):
    return D.site_xpos[sid].copy(), D.site_xmat[sid].reshape(3, 3).copy()


def neutral_offsets(M, D, rs, sites, n_neutral):
    """Constant per-segment offsets so the human's neutral pose maps exactly onto the
    robot's stand pose. Absorbs limb-length and tracker-mounting differences."""
    off = {}
    for seg, sid in sites.items():
        p_r, R_r = site_pose(M, D, sid)
        v = rs[seg]["valid"][:n_neutral]
        p_h = rs[seg]["p"][:n_neutral][v].mean(axis=0)
        q_h = R.from_quat(rs[seg]["q"][:n_neutral][v]).mean()
        off[seg] = {"dp": p_r - p_h,                                   # world-frame position offset
                    "dR": (q_h.inv() * R.from_matrix(R_r))}            # device -> robot-site rotation
    return off


def soft_knee_stand(M, D):
    """Stand keyframe with slightly bent knees (hip + knee + ankle pitch sum to 0 so the feet
    stay flat), pelvis lowered so the feet stay on the floor. Used as the IK neutral pose."""
    foot = mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_SITE, "left_foot")
    mujoco.mj_kinematics(M, D); z0 = D.site_xpos[foot, 2]
    for hip, knee, ankle in ((7, 10, 11), (13, 16, 17)):        # qpos indices, left and right leg
        D.qpos[hip] = -SOFT_KNEE / 2; D.qpos[knee] = SOFT_KNEE; D.qpos[ankle] = -SOFT_KNEE / 2
    mujoco.mj_kinematics(M, D)
    D.qpos[2] -= D.site_xpos[foot, 2] - z0
    mujoco.mj_forward(M, D)


def ik_step(M, D, targets, sites, q_stand, lo, hi, q_prev):
    """targets: {seg: (pos, rotmat or None)}. Iterates DLS IK in place on D.qpos."""
    nv = M.nv
    jacp = np.zeros((3, nv)); jacr = np.zeros((3, nv))
    for _ in range(ITERS):
        mujoco.mj_kinematics(M, D); mujoco.mj_comPos(M, D)
        rows, errs = [], []
        for seg, (p_t, R_t) in targets.items():
            sid = sites[seg]; w = TASKS[seg][3]
            p, Rm = site_pose(M, D, sid)
            mujoco.mj_jacSite(M, D, jacp, jacr, sid)
            rows.append(w * jacp); errs.append(w * (p_t - p))
            if R_t is not None:
                e_rot = R.from_matrix(R_t @ Rm.T).as_rotvec()
                rows.append(w * ROT_W * jacr); errs.append(w * ROT_W * e_rot)
        post = np.zeros((nv - 6, nv)); post[:, 6:] = np.eye(nv - 6)    # posture regularizer
        rows.append(POSTURE_W * post); errs.append(POSTURE_W * (q_stand[7:] - D.qpos[7:]))
        rows.append(SMOOTH_W * post); errs.append(SMOOTH_W * (q_prev[7:] - D.qpos[7:]))
        J = np.vstack(rows); e = np.concatenate(errs)
        dq = np.linalg.solve(J.T @ J + DAMPING * np.eye(nv), J.T @ e)
        biggest = np.abs(dq).max()
        if biggest > MAX_IK_STEP:
            dq *= MAX_IK_STEP / biggest
        mujoco.mj_integratePos(M, D.qpos, dq, 1.0)
        D.qpos[7:] = np.clip(D.qpos[7:], lo, hi)                        # joint-limit handling
    mujoco.mj_kinematics(M, D)


def main(a):
    rec = (rp.load_npz_recording(a.recording) if a.recording.endswith(".npz")
           else rp.load_recording(a.recording))
    segs = sorted(rec)
    M = build_model(segs); D = mujoco.MjData(M)
    key = mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_KEY, KEYFRAME)
    mujoco.mj_resetDataKeyframe(M, D, key); soft_knee_stand(M, D)
    q_stand = D.qpos.copy()
    cal = rp.calibrate(rec, robot_pelvis_z=float(D.qpos[2]))
    rp.apply_calibration(rec, cal)
    rs = rp.resample(rec, CONTROL_HZ)
    if not a.no_filter:
        smooth_targets(rs, CONTROL_HZ)
    sites = {s: mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_SITE, f"ik_{s}") for s in segs if s in TASKS}
    mocap = {s: M.body_mocapid[mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_BODY, f"target_{s}")] for s in segs}
    n_neutral = int(rp.NEUTRAL_S * CONTROL_HZ)
    off = neutral_offsets(M, D, rs, sites, n_neutral)
    jids = M.actuator_trnid[:, 0]
    lo, hi = M.jnt_range[jids, 0], M.jnt_range[jids, 1]

    N = len(rs["t"]); Q = np.zeros((N, M.nq)); err = {s: np.full(N, np.nan) for s in sites}
    ik = mujoco.MjData(M); ik.qpos[:] = q_stand                 # IK state, warm-started frame to frame
    last = {}
    for k in range(N):
        targets = {}
        for s in segs:
            if rs[s]["valid"][k]:
                last[s] = (rs[s]["p"][k], R.from_quat(rs[s]["q"][k]))
            if s not in last:
                continue
            p, Rq = last[s]                                       # tracking loss: hold last valid
            ik_p = p + off[s]["dp"] if s in off else p
            ik.mocap_pos[mocap[s]] = ik_p
            ik.mocap_quat[mocap[s]] = (Rq * off[s]["dR"]).as_quat()[[3, 0, 1, 2]] if s in off else [1, 0, 0, 0]
            if s in sites:
                targets[s] = (ik_p, (Rq * off[s]["dR"]).as_matrix() if TASKS[s][2] else None)
        q_prev = ik.qpos.copy()
        ik_step(M, ik, targets, sites, q_stand, lo, hi, q_prev)
        Q[k] = ik.qpos
        for s, (p_t, _) in targets.items():
            err[s][k] = np.linalg.norm(site_pose(M, ik, sites[s])[0] - p_t) * 1000
    clamped = 0
    if not a.no_filter:                       # joint velocity clamp on the OUTPUT reference only;
        step = MAX_JOINT_SPEED / CONTROL_HZ   # the IK itself keeps tracking unclamped
        for k in range(1, N):
            d = np.clip(Q[k, 7:] - Q[k - 1, 7:], -step, step)
            clamped += int(np.sum(d != Q[k, 7:] - Q[k - 1, 7:]))
            Q[k, 7:] = Q[k - 1, 7:] + d
    limit_hits = int(np.sum((Q[:, 7:] <= lo + 1e-4) | (Q[:, 7:] >= hi - 1e-4)))

    rid = Path(a.recording).stem
    RESULTS.mkdir(exist_ok=True)
    np.savez(RESULTS / f"retarget_{rid}.npz", t=rs["t"], qpos=Q, joint_targets=Q[:, 7:])
    report = {"recording": a.recording, "frames": N, "rate_hz": CONTROL_HZ,
              "ik_error_mm": {s: {"mean": float(np.nanmean(e)), "max": float(np.nanmax(e))} for s, e in err.items()},
              "joint_limit_hits": limit_hits, "velocity_clamped": clamped, "filtered": not a.no_filter,
              "max_joint_speed_rad_s": float(np.abs(np.diff(Q[:, 7:], axis=0)).max() * CONTROL_HZ),
              "p99_joint_speed_rad_s": float(np.percentile(np.abs(np.diff(Q[:, 7:], axis=0)) * CONTROL_HZ, 99))}
    (RESULTS / f"retarget_{rid}.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

    if not a.view and a.mode == "kinematic":
        return
    sim = mujoco.MjData(M); mujoco.mj_resetDataKeyframe(M, sim, key)
    sub = int(round(1 / (CONTROL_HZ * M.opt.timestep)))
    viewer_ctx = None
    if a.view:
        from mujoco import viewer as mjviewer
        viewer_ctx = mjviewer.launch_passive(M, sim)
    try:
        for k in range(N):
            t0 = time.perf_counter()
            sim.mocap_pos[:] = 0; sim.mocap_quat[:] = [1, 0, 0, 0]
            for s in segs:                                  # show the IK targets as dots
                sim.mocap_pos[mocap[s]] = ik_targets_at(k, s, rs, off, last_cache)
            if a.mode == "kinematic":
                sim.qpos[:] = Q[k]; mujoco.mj_forward(M, sim)
            else:
                sim.ctrl[:] = Q[k, 7:]
                for _ in range(sub): mujoco.mj_step(M, sim)
            if viewer_ctx:
                if not viewer_ctx.is_running(): break
                viewer_ctx.sync()
                time.sleep(max(0, 1 / CONTROL_HZ - (time.perf_counter() - t0)))
        if a.mode == "physics":
            print(f"physics mode: final pelvis height {sim.qpos[2]:.3f} m "
                  f"({'FELL' if sim.qpos[2] < 0.5 else 'standing'})")
    finally:
        if viewer_ctx: viewer_ctx.close()


last_cache = {}
def ik_targets_at(k, s, rs, off, cache):
    if rs[s]["valid"][k]:
        cache[s] = rs[s]["p"][k]
    p = cache.get(s, np.zeros(3))
    return p + off[s]["dp"] if s in off else p


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--recording", default=str(ROOT / "data" / "mock" / "mock_session.csv"))
    ap.add_argument("--mode", choices=["kinematic", "physics"], default="kinematic")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--no-filter", action="store_true", help="disable smoothing (for comparison)")
    main(ap.parse_args())
