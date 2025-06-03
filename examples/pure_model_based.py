"""Experiments 1-3 (pure model-based PT2 control): the rig's HMI plots next to the twin's prediction."""

from __future__ import annotations

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from _common import DATA, PALETTE, save

from harmonictwin import run_experiment

LABELS = {1: "symmetric load", 2: "plates, gravity not modelled", 3: "plates, model mass 11 kg"}


def main() -> None:
    fig, axes = plt.subplots(3, 3, figsize=(13, 9.2), gridspec_kw={"height_ratios": [1.15, 1, 1], "hspace": 0.45})
    for j, k in enumerate((1, 2, 3)):
        img = mpimg.imread(DATA / "hmi_plots" / f"exp{k}_control_deviation.png")
        axes[0, j].imshow(img)
        axes[0, j].axis("off")
        axes[0, j].set_title(f"Exp {k} on the rig (HMI export)\n{LABELS[k]}", loc="left", fontsize=9.5)
        sim = run_experiment(k, duration=14.0)
        cruise = (sim.t > 2.5) & (sim.t < 6.5)
        axes[1, j].plot(sim.t, sim.error, color=PALETTE["teal"], lw=1.6)
        axes[1, j].set_xlim(0, 14)
        axes[1, j].set_ylim(-0.05, 0.3)
        axes[1, j].set_ylabel("e [rad]")
        ripple = 0.5 * np.ptp(sim.error[cruise]) * 1e3
        axes[1, j].set_title(
            f"Twin: cruise {sim.error[cruise].mean():.3f} rad, ripple \u00b1{ripple:.0f} mrad", loc="left", fontsize=9.5
        )
        axes[2, j].plot(sim.t, sim.u, color=PALETTE["orange"], lw=1.4)
        axes[2, j].set_xlim(0, 14)
        axes[2, j].set_ylim(-2, 4.2)
        axes[2, j].set_ylabel("U_S [V]")
        axes[2, j].set_xlabel("time [s]")
        axes[2, j].set_title(
            f"Twin: U_S {sim.u[cruise].min():.2f}\u2026{sim.u[cruise].max():.2f} V when cruising", loc="left", fontsize=9.5
        )
        u_lo, u_hi = sim.u[cruise].min(), sim.u[cruise].max()
        print(f"exp{k}: cruise error {sim.error[cruise].mean():.4f} rad, ripple +-{ripple:.1f} mrad, U {u_lo:.2f}..{u_hi:.2f} V")
    fig.suptitle(
        "Pure model-based control: the twin reproduces the 0.25 rad lag and the gravity ripple", fontsize=12.5, fontweight="bold"
    )
    save(fig, "pure_model_based.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
