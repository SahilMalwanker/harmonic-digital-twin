"""The headline result: two controllers, six experiments, one unbalanced mass."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _common import PALETTE, save

from harmonictwin import (
    EXPERIMENTS,
    LAB_PROFILE,
    RIG,
    controller_for,
    identify_unbalance,
    load_measurements,
    ripple_gain,
    ripple_phasor,
    run_experiment,
)


def main() -> None:
    meas = load_measurements()
    phasors = {k: ripple_phasor(meas.t, e, LAB_PROFILE)[0] for k, e in meas.error.items()}
    fits = {}
    for name, runs in (("P-PI (exp 4-6)", (4, 5, 6)), ("P-ReDuS (exp 7-9)", (7, 8, 9))):
        est = identify_unbalance([EXPERIMENTS[k].model_mass for k in runs], [phasors[k] for k in runs])
        fits[name] = (runs, est)
        offset = np.rad2deg(est.offset)
        print(f"{name}: m = {est.mass:.2f} kg, gravity offset = {offset:.2f} deg, line misfit {est.residual * 1e6:.1f} urad")

    fig = plt.figure(figsize=(13, 4.9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.35], hspace=0.55, wspace=0.22)
    ax = fig.add_subplot(gs[:, 0])
    colours = {"P-PI (exp 4-6)": PALETTE["orange"], "P-ReDuS (exp 7-9)": PALETTE["teal"]}
    m_axis = np.linspace(0, 20, 400)
    curve = np.abs(RIG.m * np.exp(1j * RIG.gravity_offset) - m_axis)
    ax.plot(m_axis, curve, color=PALETTE["ink"], lw=1.4, label=r"twin: $|m\,e^{i\varphi} - m_{model}|$")
    styles = {"P-PI (exp 4-6)": ("o", 110), "P-ReDuS (exp 7-9)": ("D", 45)}
    for name, (runs, est) in fits.items():
        marker, size = styles[name]
        for k in runs:
            unit_gain = abs(ripple_gain(controller_for(k), RIG, LAB_PROFILE.v_m))
            ax.scatter(
                EXPERIMENTS[k].model_mass,
                abs(phasors[k]) / unit_gain,
                s=size,
                marker=marker,
                color=colours[name],
                edgecolor=PALETTE["ink"],
                zorder=5,
            )
        ax.scatter(
            [], [], s=size, marker=marker, color=colours[name], edgecolor=PALETTE["ink"], label=f"{name}: m = {est.mass:.2f} kg"
        )
    for k_pi, k_rd in ((4, 7), (5, 8), (6, 9)):
        y = abs(phasors[k_pi]) / abs(ripple_gain(controller_for(k_pi), RIG, LAB_PROFILE.v_m))
        ax.annotate(
            f"exp {k_pi} / {k_rd}", (EXPERIMENTS[k_pi].model_mass, y), xytext=(9, 6), textcoords="offset points", fontsize=8
        )
    best = RIG.m * np.cos(RIG.gravity_offset)
    ax.axvline(best, color=PALETTE["muted"], lw=0.9, ls=(0, (4, 3)))
    ax.text(best + 0.3, 11.5, f"gravity cancelled\nat m \u2248 {best:.1f} kg", fontsize=8.5, color=PALETTE["ink"])
    ax.set_xlim(-0.5, 20)
    ax.set_ylim(0, 16)
    ax.set_xlabel("mass entered in the controller's model  m_model  [kg]")
    ax.set_ylabel("measured ripple / twin loop gain  [kg]")
    ax.set_title("Uncompensated unbalance seen by each experiment", loc="left")
    ax.legend(loc="upper right", fontsize=8)

    for row, k in enumerate((6, 9)):
        axr = fig.add_subplot(gs[row, 1])
        sim = run_experiment(k)
        m = (meas.t > 2.0) & (meas.t < 6.7)
        mean_meas = ripple_phasor(meas.t, meas.error[k], LAB_PROFILE)[1]
        twin = np.interp(meas.t, sim.t, sim.error)
        axr.plot(meas.t[m], (meas.error[k][m] - mean_meas) * 1e3, color=PALETTE["measured"], lw=2.2, label="measured on the rig")
        axr.plot(meas.t[m], (twin[m] - mean_meas) * 1e3, color=PALETTE["teal"], lw=1.4, ls=(0, (5, 2)), label="digital twin")
        axr.set_ylabel("e \u2212 v/K_L  [mrad]")
        axr.set_title(f"Exp {k}: {EXPERIMENTS[k].description}", loc="left", fontsize=9.5)
        axr.legend(loc="upper right", ncol=2)
    axr.set_xlabel("time [s]")
    fig.suptitle(
        f"Both cascades independently identify the unbalanced load: {RIG.m:.1f} kg (model assumed 9.4\u201311 kg)",
        fontsize=12.5,
        fontweight="bold",
    )
    save(fig, "identification.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
