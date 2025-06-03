"""Measured control deviation of experiments 4-9 next to the digital twin, with the residual."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _common import PALETTE, save

from harmonictwin import EXPERIMENTS, load_measurements, run_experiment


def main() -> None:
    meas = load_measurements()
    fig, axes = plt.subplots(
        4, 3, figsize=(13, 8.6), sharex=True, gridspec_kw={"height_ratios": [2.2, 1, 2.2, 1], "hspace": 0.25}
    )
    for col_block, runs in enumerate(((4, 5, 6), (7, 8, 9))):
        for j, k in enumerate(runs):
            ax, axr = axes[2 * col_block, j], axes[2 * col_block + 1, j]
            sim = run_experiment(k)
            twin = np.interp(meas.t, sim.t, sim.error)
            late = meas.t >= 0.25
            rms = np.sqrt(np.mean((twin[late] - meas.error[k][late]) ** 2)) * 1e3
            ax.plot(meas.t, meas.error[k] * 1e3, color=PALETTE["measured"], lw=3.0, label="measured")
            ax.plot(meas.t, twin * 1e3, color=PALETTE["teal"], lw=1.3, ls=(0, (5, 2)), label="digital twin")
            scheme = "P-PI" if EXPERIMENTS[k].controller == "pi" else "P-ReDuS"
            ax.set_title(f"Exp {k}: {scheme}, model mass {EXPERIMENTS[k].model_mass:g} kg", loc="left", fontsize=9.5)
            ax.text(
                0.98,
                0.5,
                f"RMS difference\n{rms:.2f} mrad",
                transform=ax.transAxes,
                ha="right",
                fontsize=8.5,
                color=PALETTE["ink"],
            )
            axr.plot(meas.t, (meas.error[k] - twin) * 1e3, color=PALETTE["orange"], lw=0.9)
            axr.axhline(0, color=PALETTE["muted"], lw=0.7)
            axr.set_ylim(-5, 5)
            if j == 0:
                ax.set_ylabel("e = q_d \u2212 q [mrad]")
                axr.set_ylabel("measured \u2212\ntwin [mrad]")
            if col_block == 0 and j == 0:
                ax.legend(loc="lower center")
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    fig.suptitle(
        "One twin, six recordings: control deviation measured at 1 kHz vs. simulation", fontsize=12.5, fontweight="bold", y=0.95
    )
    save(fig, "twin_vs_measured.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
