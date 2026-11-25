#!/usr/bin/env python3
"""Two modules on the tetra connector, driven by phase oscillators.

Model: models/xml/colony/test_2_tetra.xml (the colony rotates about one shaft).

Each module i has a phase phi_i. The motor follows a cosine target with
torque PD control, and the passive paddle angle q_i gives a Flex sensor value

    S_i = sign * clip(q_i / (2 q_limit), -0.5, 0.5)       (sign = -1)

Phase update (own sensor feedback, Nishikawa-type):

    d phi_i / dt = omega_i + sigma * Z(phi_i) * S_i

sigma = 0 (default) is open loop. Replace z_profile() to try another Z.

Usage:
    python src/controller/scripts/run_two_module_cpg.py
    python src/controller/scripts/run_two_module_cpg.py --sigma 0.1 --duration 60
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MODEL = ROOT / "models" / "xml" / "colony" / "test_2_tetra.xml"

# Settings used with the Kurabayashi Lab CAD model
AMPLITUDE_DEG = 40.0   # motor target amplitude (Kim motor_hinge range is +/-45 deg)
MOTOR_KP = 0.060       # torque PD gains [N m / rad], [N m s / rad]
MOTOR_KD = 0.002
SENSOR_SIGN = -1       # Flex sensor polarity (S = -q / (2 q_limit))


def z_profile(phi: float) -> float:
    """Phase response used in the feedback term. Example only."""
    return math.sin(phi)


def flex_sensor(q: float, q_limit: float) -> float:
    return SENSOR_SIGN * float(np.clip(q / (2.0 * q_limit), -0.5, 0.5))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    p.add_argument("--duration", type=float, default=30.0, help="simulation time [s]")
    p.add_argument("--omega1", type=float, default=2.5 * math.pi, help="module 1 angular frequency [rad/s]")
    p.add_argument("--omega2", type=float, default=2.4 * math.pi, help="module 2 angular frequency [rad/s]")
    p.add_argument("--sigma", type=float, default=0.0, help="feedback gain (0 = open loop)")
    p.add_argument("--phi0", type=float, nargs=2, default=(0.0, 0.0), help="initial phases [rad]")
    p.add_argument("--log-every", type=int, default=10, help="save every N physics steps")
    p.add_argument("--out", type=Path, default=ROOT / "logs" / "two_module_cpg.csv")
    p.add_argument("--view", action="store_true", help="open the MuJoCo viewer (use mjpython on macOS)")
    args = p.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    motor = [(model.joint(f"r{i}_motor_hinge").qposadr[0], model.joint(f"r{i}_motor_hinge").dofadr[0]) for i in (1, 2)]
    paddle = [model.joint(f"r{i}_paddle_hinge").qposadr[0] for i in (1, 2)]
    actuator = [model.actuator(f"r{i}_motor_tau").id for i in (1, 2)]
    q_limit = float(model.jnt_range[model.joint("r1_paddle_hinge").id, 1])
    yaw = model.joint("colony_yaw").qposadr[0]

    amplitude = math.radians(AMPLITUDE_DEG)
    omega = (args.omega1, args.omega2)
    phi = list(args.phi0)
    rows = []

    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    for step in range(int(round(args.duration / dt))):
        s = [flex_sensor(data.qpos[paddle[i]], q_limit) for i in range(2)]
        if step % args.log_every == 0:
            rows.append([data.time, *phi, *(data.qpos[motor[i][0]] for i in range(2)),
                         *(data.qpos[paddle[i]] for i in range(2)), *s, data.qpos[yaw]])
        for i in range(2):
            phi[i] += dt * (omega[i] + args.sigma * z_profile(phi[i]) * s[i])
            q, v = motor[i]
            torque = MOTOR_KP * (amplitude * math.cos(phi[i]) - data.qpos[q]) - MOTOR_KD * data.qvel[v]
            data.ctrl[actuator[i]] = np.clip(torque, *model.actuator_ctrlrange[actuator[i]])
        mujoco.mj_step(model, data)
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "phi1_rad", "phi2_rad", "motor1_rad", "motor2_rad",
                    "paddle1_rad", "paddle2_rad", "S1", "S2", "colony_yaw_rad"])
        w.writerows(rows)

    dphi = (phi[1] - phi[0] + math.pi) % (2 * math.pi) - math.pi
    print(f"MuJoCo {mujoco.__version__}, {data.time:.1f} s simulated, sigma = {args.sigma:g}")
    print(f"final phase difference phi2 - phi1 = {dphi / math.pi:+.3f} pi rad, colony yaw = {math.degrees(data.qpos[yaw]):+.1f} deg")
    print(f"log: {args.out}")


if __name__ == "__main__":
    main()
