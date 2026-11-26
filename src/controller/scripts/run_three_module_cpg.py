#!/usr/bin/env python3
"""Drive three CAD paddles mounted 120 degrees apart on a ring.

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
        __doc__, "test_3_ring.xml", "three_module_cpg.csv",
        omega=(2.5 * math.pi, 2.4 * math.pi, 2.3 * math.pi),
        phi0=(0.0, math.tau / 3, 2 * math.tau / 3),
    )


if __name__ == "__main__":
    main()
