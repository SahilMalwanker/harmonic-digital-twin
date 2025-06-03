"""Closing the gap: what feed-forward and an identified model do to the 204 mrad lag."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _common import PALETTE, save

from harmonictwin import LAB_PROFILE, RIG, load_measurements, make_controller, plant_for, run_experiment, simulate


def main() -> None:
    meas = load_measurements()
    identified = RIG  # model = identified twin (mass, gravity offset, Coulomb friction)
    cases = [
        ("measured on the rig (exp 7)", meas.t, meas.error[7], PALETTE["measured"], 3.0),
    ]
    sims = {
        "twin, as run in the lab (K_V = 0)": run_experiment(7),
        "+ velocity pre-control K_V = 1": run_experiment(7, K_V=1.0),
        "+ acceleration feed-forward, identified model": simulate(
            make_controller("redus", identified, K_V=1.0, K_A=1.0), plant_for(7), LAB_PROFILE
        ),
        "computed torque with the identified model": simulate(
            make_controller("pt2", identified, feedforward=True), plant_for(7), LAB_PROFILE
        ),
    }
    colours = [PALETTE["teal"], PALETTE["blue"], PALETTE["purple"], PALETTE["orange"]]
    for (name, sim), c in zip(sims.items(), colours, strict=True):
        cases.append((name, sim.t, sim.error, c, 1.6))

    fig, (ax, axb) = plt.subplots(1, 2, figsize=(13, 4.3), gridspec_kw={"width_ratios": [1.6, 1]})
    peaks = []
    for name, t, e, c, lw in cases:
        peak = np.abs(e[t >= 0.25]).max() * 1e3
        peaks.append((name, peak, c))
        ax.semilogy(t, np.maximum(np.abs(e) * 1e3, 1e-3), color=c, lw=lw, label=f"{name}  (peak {peak:.3g} mrad)")
        print(f"{name:52s} peak |e| = {peak:8.3f} mrad")
    ax.set_ylim(1e-2, 400)
    ax.set_xlim(0, 12.25)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|e| = |q_d \u2212 q|  [mrad]")
    ax.set_title("Same ramp (1.5 rad/s, 10 rad), same P-ReDuS gains, different feed-forward", loc="left")
    ax.legend(loc="upper right", fontsize=7.5, facecolor="white", frameon=True, framealpha=0.95)
    names = ["rig", "twin", "+ K_V", "+ K_A &\nmodel", "computed\ntorque"]
    axb.bar(names, [p for _, p, _ in peaks], color=[c for _, _, c in peaks])
    axb.set_yscale("log")
    for i, (_, p, _) in enumerate(peaks):
        axb.text(i, p * 1.25, f"{p:.3g}", ha="center", fontsize=8.5)
    axb.set_ylabel("peak |e| after 0.25 s [mrad]")
    axb.set_title("Peak tracking error", loc="left")
    axb.set_ylim(1e-2, 1e3)
    fig.suptitle(
        "The 204 mrad error is the P position loop's ramp lag, and feed-forward removes it", fontsize=12.5, fontweight="bold"
    )
    fig.tight_layout()
    save(fig, "closing_the_gap.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
