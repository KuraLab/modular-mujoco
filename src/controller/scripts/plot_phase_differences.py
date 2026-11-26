#!/usr/bin/env python3
"""Save a plot of relative oscillator phases from a CPG CSV log."""
from __future__ import annotations

import argparse
from pathlib import Path
import re

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import numpy as np

LOG_DIR = Path(__file__).resolve().parents[3] / "logs"
DEMO_LOGS = {
    2: "two_module_cpg.csv",
    3: "three_module_cpg.csv",
    4: "four_module_cpg.csv",
}


def wrapped_phase_series(time_s: np.ndarray, phase_delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Wrap to [-pi, pi) and break the line where the phase crosses the boundary."""
    period = 2 * np.pi
    wrapped = (phase_delta + np.pi) % period - np.pi
    # Stabilize the boundary for phase differences that equal pi up to roundoff.
    at_boundary = np.isclose(np.abs(wrapped), np.pi, rtol=0, atol=1e-10)
    wrapped[at_boundary] = -np.pi

    turns = np.rint((phase_delta - wrapped) / period)
    breaks = np.flatnonzero(turns[1:] != turns[:-1]) + 1
    return np.insert(time_s, breaks, time_s[breaks]), np.insert(wrapped, breaks, np.nan)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("csv", type=Path, nargs="?", help="path to a CPG log")
    source.add_argument("--modules", type=int, choices=DEMO_LOGS,
                        help="select the default log by module count (default is 4)")
    parser.add_argument("--reference", type=int, default=1, help="reference module number (default is 1)")
    parser.add_argument("--out", type=Path, help="output path (default is <CSV stem>_phase_ref<module>.png beside the CSV)")
    args = parser.parse_args()
    csv_path = args.csv or LOG_DIR / DEMO_LOGS[args.modules or 4]

    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, ndmin=1)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    columns = data.dtype.names or ()
    if "time_s" not in columns or data.size == 0:
        parser.error("the CSV must contain time_s and at least one data row")

    phases = {}
    for name in columns:
        match = re.fullmatch(r"phi([1-9][0-9]*)_rad", name)
        if match:
            phases[int(match[1])] = data[name]
    if len(phases) < 2:
        parser.error("the CSV must contain phase columns for at least two modules")
    if args.reference not in phases:
        available = ", ".join(str(module) for module in sorted(phases))
        parser.error(f"reference module {args.reference} is missing from the CSV (available modules are {available})")

    fig = Figure(figsize=(9, 4.8), constrained_layout=True)
    FigureCanvasAgg(fig)
    ax = fig.subplots()
    ax.axhline(0, color="0.5", linewidth=0.9, linestyle="--")
    reference = phases[args.reference]
    for module, phase in sorted(phases.items()):
        if module == args.reference:
            continue
        times, relative_rad = wrapped_phase_series(data["time_s"], phase - reference)
        ax.plot(times, relative_rad, label=f"Module {module}",
                marker="o" if data.size == 1 else None, linewidth=1.6, zorder=3)

    ax.set_title(f"Phase difference from module {args.reference}")
    ax.set_xlabel("Simulation time [s]")
    ax.set_ylabel("Relative phase [rad]")
    ax.set_ylim(-np.pi, np.pi)
    ax.set_yticks([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi],
                  labels=[r"$-\pi$", r"$-\pi/2$", "0", r"$\pi/2$", r"$\pi$"])
    ax.grid(alpha=0.25)
    ax.legend()

    output = args.out or csv_path.with_name(f"{csv_path.stem}_phase_ref{args.reference}.png")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
