"""How much does a wrong model hurt? Sweep the model mass for all three control schemes on the twin."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _common import PALETTE, save

from harmonictwin import LAB_MODEL, LAB_PROFILE, RIG, make_controller, ripple_phasor, simulate


def main() -> None:
    masses = np.linspace(0, 25, 26)
    kinds = {
        "pt2": ("model-based PT2", PALETTE["purple"]),
        "pi": ("P-PI cascade", PALETTE["orange"]),
        "redus": ("P-ReDuS cascade", PALETTE["teal"]),
    }
    ripple = {k: [] for k in kinds}
    final = {k: [] for k in kinds}
    for kind in kinds:
        for m in masses:
            sim = simulate(make_controller(kind, LAB_MODEL.with_(m=m)), RIG, LAB_PROFILE, duration=12.0)
            ripple[kind].append(abs(ripple_phasor(sim.t, sim.error, LAB_PROFILE)[0]) * 1e3)
            final[kind].append(abs(sim.error[sim.t > 11.0].mean()) * 1e3)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(12.5, 4.0))
    for kind, (label, c) in kinds.items():
        ax0.semilogy(masses, ripple[kind], color=c, lw=2, marker="o", ms=3.5, label=label)
        ax1.semilogy(masses, np.maximum(final[kind], 1e-3), color=c, lw=2, marker="o", ms=3.5, label=label)
        i = int(np.argmin(ripple[kind]))
        print(
            f"{label:16s} ripple at m=0: {ripple[kind][0]:6.2f} mrad, at m=11: {ripple[kind][11]:6.2f}, best m={masses[i]:.0f} kg"
        )
    for ax in (ax0, ax1):
        ax.axvline(RIG.m, color=PALETTE["muted"], lw=0.9, ls=(0, (4, 3)))
        ax.text(RIG.m + 0.3, ax.get_ylim()[1] * 0.4, f"identified\n{RIG.m:.1f} kg", fontsize=8)
        for m_lab in (0, 11, 16):
            ax.axvline(m_lab, color=PALETTE["grid"], lw=4, zorder=0)
        ax.set_xlabel("mass in the controller's model [kg]   (grey bands: values used in the lab)")
    ax0.set_ylabel("gravity ripple amplitude [mrad]")
    ax0.set_title("Ripple while cruising at 1.5 rad/s", loc="left")
    ax1.set_ylabel("|e| after the motion [mrad]")
    ax1.set_title("Error left after stopping", loc="left")
    ax0.legend()
    fig.suptitle(
        "Integral action forgives a wrong model at standstill, but only an accurate model removes the ripple",
        fontsize=12,
        fontweight="bold",
    )
    fig.tight_layout()
    save(fig, "robustness.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
