#!/usr/bin/env python3
"""Two modules on the tetra connector, driven by phase oscillators.

Model: models/xml/colony/test_2_tetra.xml (the colony rotates about one shaft).

Each module i has a phase phi_i. The motor follows a cosine target with
torque PD control, and the passive paddle angle q_i gives a Flex sensor value

    S_i = sign * clip(q_i / (2 q_limit), -0.5, 0.5)       (sign = -1)

Phase update (each module uses its own Flex sensor value):

    d phi_i / dt = omega_i + sigma * Z(phi_i) * S_i

sigma = 0 (default) is open loop. Edit z_profile() in module_cpg.py to try another Z.

Usage:
    python src/controller/scripts/run_two_module_cpg.py
    python src/controller/scripts/run_two_module_cpg.py --sigma 0.1 --duration 60
"""
from __future__ import annotations

import math

import mujoco

if __package__:
    from .module_cpg import cpg_parser, run_cpg
else:
    from module_cpg import cpg_parser, run_cpg


def main() -> None:
    parser = cpg_parser(
        __doc__, "test_2_tetra.xml", "two_module_cpg.csv",
        omega=(2.5 * math.pi, 2.4 * math.pi), phi0=(0.0, 0.0),
    )
    args = parser.parse_args()
    elapsed, phi, angles = run_cpg(args)
    yaw = angles["yaw"]

    dphi = (phi[1] - phi[0] + math.pi) % (2 * math.pi) - math.pi
    print(f"MuJoCo {mujoco.__version__}, {elapsed:.1f} s simulated, sigma = {args.sigma:g}")
    print(f"final phase difference phi2 - phi1 = {dphi / math.pi:+.3f} pi rad, colony yaw = {math.degrees(yaw):+.1f} deg")
    print(f"log: {args.out}")


if __name__ == "__main__":
    main()
