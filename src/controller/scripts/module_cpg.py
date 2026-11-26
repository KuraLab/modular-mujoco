"""Shared torque PD and phase oscillator control for the CAD colony demos."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import csv
import math
from pathlib import Path
import random
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[3]

# Settings used with the Kurabayashi Lab CAD model
AMPLITUDE_DEG = 40.0   # motor target amplitude (motor_hinge range is +/-45 deg)
MOTOR_KP = 0.060       # torque PD gains [N m / rad], [N m s / rad]
MOTOR_KD = 0.002
MOTOR_MAX_SPEED = math.radians(60.0) / 0.18  # HS-5086WP at 4.8 V [rad/s]
SENSOR_SIGN = -1       # Flex sensor polarity (S = -q / (2 q_limit))
VIEW_SYNC_INTERVAL = 1.0 / 60.0
FLOAT_DRAFT = 0.015          # Water level above the ring center at rest [m]
HEAVE_PERIOD = 2.0          # Nominal period from the restoring force alone [s]
HEAVE_DAMPING_RATIO = 0.25


def z_profile(phi: float) -> float:
    """Phase response used in the feedback term. Example only."""
    return -math.cos(phi)


def flex_sensor(q: float, q_limit: float) -> float:
    return SENSOR_SIGN * float(np.clip(q / (2.0 * q_limit), -0.5, 0.5))


def colony_angles(qpos: np.ndarray) -> tuple[float, ...]:
    """Return hinge yaw or quaternion yaw, pitch and roll in ZYX order."""
    if qpos.size == 1:
        return (float(qpos[0]),)

    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, qpos)
    r = rotation.reshape(3, 3)
    horizontal = math.hypot(r[0, 0], r[1, 0])
    pitch = math.atan2(-r[2, 0], horizontal)
    if horizontal < 1e-8:
        # At vertical pitch, choose the equivalent orientation with zero roll.
        return math.atan2(-r[0, 1], r[1, 1]), pitch, 0.0
    yaw = math.atan2(r[1, 0], r[0, 0])
    roll = math.atan2(r[2, 1], r[2, 2])
    return yaw, pitch, roll


def cpg_parser(
    description: str, model_file: str, csv_file: str,
    omega: tuple[float, ...], phi0: tuple[float, ...],
    *, allow_random_phi0: bool = False,
) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", type=Path, default=ROOT / "models" / "xml" / "colony" / model_file)
    p.add_argument("--duration", type=float, default=30.0, help="simulation time [s]")
    for i, frequency in enumerate(omega, start=1):
        p.add_argument(f"--omega{i}", type=float, default=frequency, help=f"module {i} angular frequency [rad/s]")
    p.add_argument("--sigma", type=float, default=0.0, help="feedback gain (0 = open loop)")
    phase_options = p.add_mutually_exclusive_group() if allow_random_phi0 else p
    phase_options.add_argument("--phi0", type=float, nargs=len(phi0), default=phi0, help="initial phases [rad] in module order")
    if allow_random_phi0:
        phase_options.add_argument("--random-phi0", action="store_true", help="draw each initial phase uniformly from [0, 2*pi) rad")
        p.add_argument("--seed", type=int, help="seed for --random-phi0")
    p.add_argument("--log-every", type=int, default=10, help="save every N physics steps")
    p.add_argument("--out", type=Path, default=ROOT / "logs" / csv_file)
    p.add_argument("--view", action="store_true", help="open the MuJoCo viewer (use mjpython on macOS)")
    return p


def load_cpg_model(path: Path, fixed_center: bool, *, float_at_surface: bool = False) -> mujoco.MjModel:
    if not fixed_center and not float_at_surface:
        return mujoco.MjModel.from_xml_path(str(path))

    spec = mujoco.MjSpec.from_file(str(path))
    joint = spec.joint("colony_orientation")
    if joint is None:
        joint = spec.joint("colony_yaw")
    if float_at_surface:
        if joint is None or joint.type != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("--float requires a free colony joint")
        colony = joint.parent
        start = colony.pos.copy()
        # Place the half-turn at the colony origin so reset also starts face up.
        flip = spec.worldbody.add_frame(pos=start, quat=[0, 1, 0, 0])
        if colony.frame is not None:
            flip.set_frame(colony.frame)
        colony.set_frame(flip)
        colony.pos[:] = 0
        spec.worldbody.add_geom(
            name="water_surface", type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=[0, 0, start[2] + FLOAT_DRAFT], size=[0, 0, 0.05],
            rgba=[0.22, 0.56, 0.76, 0.22], contype=0, conaffinity=0,
        )
    elif joint is not None and joint.type == mujoco.mjtJoint.mjJNT_FREE:
        joint.type = mujoco.mjtJoint.mjJNT_BALL
    return spec.compile()


def run_cpg(
    args: argparse.Namespace, *, fixed_center: bool = False, follow_colony: bool = False,
    float_at_surface: bool = False,
) -> tuple[float, list[float], dict[str, float]]:
    """Run and log the demo, then return its elapsed time, phases, and orientation angles."""
    model = load_cpg_model(args.model, fixed_center, float_at_surface=float_at_surface)
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    max_motor_step = MOTOR_MAX_SPEED * dt

    phi = list(args.phi0)
    modules = range(1, len(phi) + 1)
    omega = [getattr(args, f"omega{i}") for i in modules]
    motor_joints = [model.joint(f"r{i}_motor_hinge") for i in modules]
    motor_qpos = [joint.qposadr[0] for joint in motor_joints]
    motor_dofs = [joint.dofadr[0] for joint in motor_joints]
    paddle_joints = [model.joint(f"r{i}_paddle_hinge") for i in modules]
    paddle = [joint.qposadr[0] for joint in paddle_joints]
    actuator = [model.actuator(f"r{i}_motor_tau").id for i in modules]
    q_limit = float(paddle_joints[0].range[1])  # Same paddle limits on all CAD copies
    colony_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "colony_orientation")
    if colony_joint == -1:
        colony_joint = model.joint("colony_yaw").id
    colony_qpos = data.joint(colony_joint).qpos
    if colony_qpos.size == 7:
        position = colony_qpos[:3]
        orientation = colony_qpos[3:]
        position_columns = ("colony_x_m", "colony_y_m", "colony_z_m")
    else:
        position = ()
        orientation = colony_qpos
        position_columns = ()
    angle_names = ("yaw", "pitch", "roll") if orientation.size == 4 else ("yaw",)
    if float_at_surface:
        # The free joint's first three DOFs translate along the world axes.
        z_dof = int(model.jnt_dofadr[colony_joint]) + 2
        resting_z = float(position[2])
        mass = model.body_subtreemass[model.jnt_bodyid[colony_joint]]
        heave_omega = math.tau / HEAVE_PERIOD
        heave_stiffness = mass * heave_omega**2
        heave_damping = 2 * HEAVE_DAMPING_RATIO * mass * heave_omega
    amplitude = math.radians(AMPLITUDE_DEG)
    rows = []

    viewer_context = nullcontext()
    camera_target = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer_context = mj_viewer.launch_passive(model, data)
        if follow_colony:
            camera_target = data.xpos[model.jnt_bodyid[colony_joint]]

    step_count = int(round(args.duration / dt))
    with viewer_context as viewer:
        if viewer is not None:
            playback_start = time.monotonic()
            next_sync = playback_start + VIEW_SYNC_INTERVAL
        for step in range(step_count):
            if viewer is not None and not viewer.is_running():
                break

            s = [flex_sensor(data.qpos[q], q_limit) for q in paddle]
            motor_before = data.qpos[motor_qpos]
            if step % args.log_every == 0:
                rows.append([data.time, *phi, *motor_before,
                             *(data.qpos[q] for q in paddle), *s, *colony_angles(orientation), *position])
            for i, (q, v) in enumerate(zip(motor_qpos, motor_dofs)):
                phi[i] += dt * (omega[i] + args.sigma * z_profile(phi[i]) * s[i])
                torque = MOTOR_KP * (amplitude * math.cos(phi[i]) - data.qpos[q]) - MOTOR_KD * data.qvel[v]
                data.ctrl[actuator[i]] = np.clip(torque, *model.actuator_ctrlrange[actuator[i]])
            if float_at_surface:
                data.qfrc_applied[z_dof] = -heave_stiffness * (position[2] - resting_z) - heave_damping * data.qvel[z_dof]
            mujoco.mj_step(model, data)
            # Limit the integrated angle change as well as the speed retained for the next step.
            data.qpos[motor_qpos] = np.clip(
                data.qpos[motor_qpos], motor_before - max_motor_step, motor_before + max_motor_step,
            )
            data.qvel[motor_dofs] = np.clip(data.qvel[motor_dofs], -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED)

            if viewer is not None:
                # Pace playback against the start time so individual delays don't accumulate.
                step_deadline = playback_start + (step + 1) * dt
                now = time.monotonic()
                if max(step_deadline, now) < next_sync and step + 1 < step_count:
                    continue
                remaining = step_deadline - now
                if remaining > 0:
                    time.sleep(remaining)
                mujoco.mj_kinematics(model, data)
                if camera_target is not None:
                    with viewer.lock():
                        viewer.cam.lookat[:] = camera_target
                viewer.sync()
                next_sync = time.monotonic() + VIEW_SYNC_INTERVAL

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", *(f"phi{i}_rad" for i in modules), *(f"motor{i}_rad" for i in modules),
                    *(f"paddle{i}_rad" for i in modules), *(f"S{i}" for i in modules),
                    *(f"colony_{name}_rad" for name in angle_names), *position_columns])
        w.writerows(rows)

    return data.time, phi, dict(zip(angle_names, colony_angles(orientation)))


def run_ring_cpg(
    description: str, model_file: str, csv_file: str,
    omega: tuple[float, ...], phi0: tuple[float, ...],
) -> None:
    """Parse the ring demos' options and run the shared controller."""
    parser = cpg_parser(description, model_file, csv_file, omega, phi0, allow_random_phi0=True)
    parser.add_argument("--omega", type=float, help="set all natural angular frequencies [rad/s], overriding individual values")
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument("--fixed-center", action="store_true", help="pin the center while allowing rotation about all three axes")
    motion.add_argument("--float", dest="float_at_surface", action="store_true", help="apply a vertical restoring force near the water surface")
    args = parser.parse_args()
    if args.omega is not None:
        for i in range(1, len(omega) + 1):
            setattr(args, f"omega{i}", args.omega)

    if args.random_phi0:
        rng = random.Random(args.seed)
        args.phi0 = tuple(rng.random() * math.tau for _ in phi0)
        print(f"initial phases [rad] = {args.phi0}")
    elif args.seed is not None:
        parser.error("--seed requires --random-phi0")

    elapsed, _, angles = run_cpg(
        args, fixed_center=args.fixed_center, follow_colony=True, float_at_surface=args.float_at_surface,
    )
    print(f"MuJoCo {mujoco.__version__}, {elapsed:.1f} s simulated, sigma = {args.sigma:g}")
    print("colony " + ", ".join(f"{axis} = {math.degrees(angle):+.1f} deg" for axis, angle in angles.items()))
    print(f"log: {args.out}")
