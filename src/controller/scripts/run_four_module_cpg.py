#!/usr/bin/env python3
"""Drive four CAD paddles through vertical strokes with phase oscillators.

The ring can translate and rotate freely. Pass --float to let it bob near
the water surface or --fixed-center to hold its center in place. Pass --sigma
to enable feedback from each passive paddle.
"""
from __future__ import annotations

import math

if __package__:
    from .module_cpg import run_ring_cpg
else:
    from module_cpg import run_ring_cpg


def main() -> None:
    run_ring_cpg(
        __doc__, "test_4_ring.xml", "four_module_cpg.csv",
        omega=(2.5 * math.pi, 2.4 * math.pi, 2.3 * math.pi, 2.2 * math.pi),
        phi0=(0.0, math.pi / 2, math.pi, 3 * math.pi / 2),
    )


if __name__ == "__main__":
    main()
