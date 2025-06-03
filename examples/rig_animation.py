"""Hero animation: the joint rig sweeping 10 rad while the twin replays the measured control deviation."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _common import MEDIA, PALETTE, save_gif
from matplotlib.patches import Circle, Polygon

from harmonictwin import LAB_PROFILE, RIG, load_measurements, ripple_phasor, run_experiment

EXP = 6
STEP = 0.1  # seconds of experiment per animation frame (2x real time at 20 fps)


def bar(angle: float, r0: float, r1: float, width: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    n = np.array([-s, c]) * width / 2
    a, b = np.array([c, s]) * r0, np.array([c, s]) * r1
    return np.array([a + n, b + n, b - n, a - n])


def main() -> None:
    meas = load_measurements()
    sim = run_experiment(EXP)
    t = meas.t
    e_meas = meas.error[EXP]
    e_twin = np.interp(t, sim.t, sim.error)
    q_d = LAB_PROFILE.sample(t)[0]
    q = q_d - e_meas
    lag = ripple_phasor(t, e_meas, LAB_PROFILE)[1]
    show = -RIG.gravity_offset  # draw the load horizontal (maximum gravity torque) at q = -offset

    fig = plt.figure(figsize=(11, 4.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[0.9, 1.4], hspace=0.55, wspace=0.12, left=0.02, right=0.98, top=0.86, bottom=0.12)
    fig.suptitle(
        f"Exp {EXP} (P-PI cascade, gravity not modelled): rig measurement vs. digital twin", fontsize=12, fontweight="bold"
    )
    ax = fig.add_subplot(gs[:, 0])
    ax.set_aspect("equal")
    ax.set_xlim(-1.25, 1.25)
    ax.set_ylim(-1.25, 1.25)
    ax.axis("off")
    ax.add_patch(Circle((0, 0), 1.08, fill=False, ec=PALETTE["grid"], lw=1.2))
    ghost = Polygon(bar(0, 0, 1.0, 0.05), closed=True, fill=False, ec=PALETTE["orange"], lw=1.5, ls=(0, (3, 2)))
    link = Polygon(bar(0, -0.25, 0.95, 0.14), closed=True, fc=PALETTE["measured"], ec=PALETTE["ink"], lw=1.2)
    plates = Polygon(bar(0, 0.62, 0.92, 0.32), closed=True, fc=PALETTE["teal"], ec=PALETTE["ink"], lw=1.2)
    for patch in (ghost, link, plates):
        ax.add_patch(patch)
    ax.add_patch(Circle((0, 0), 0.16, fc=PALETTE["ink"], ec="none"))
    ax.annotate("", xy=(0, -1.2), xytext=(0, -0.95), arrowprops={"arrowstyle": "-|>", "color": PALETTE["red"], "lw": 1.6})
    ax.text(0.05, -1.17, "g", color=PALETTE["red"], fontsize=11, fontweight="bold")
    label = ax.text(-1.22, 1.15, "", fontsize=10, va="top", color=PALETTE["ink"])
    ax.text(-1.22, -1.0, "dashed: set-point q_d\nsolid: measured joint q", fontsize=8, color=PALETTE["muted"])

    axe = fig.add_subplot(gs[0, 1])
    axr = fig.add_subplot(gs[1, 1])
    (m1,) = axe.plot([], [], color=PALETTE["measured"], lw=3.0, label="measured (1 kHz)")
    (s1,) = axe.plot([], [], color=PALETTE["teal"], lw=1.3, ls=(0, (5, 2)), label="digital twin")
    axe.set_xlim(0, 12.25)
    axe.set_ylim(-10, 230)
    axe.set_ylabel("e = q_d \u2212 q [mrad]")
    axe.legend(loc="upper right", ncol=2)
    axe.set_title(f"Control deviation: the lag v/K_L = {lag * 1e3:.0f} mrad dominates", loc="left", fontsize=10)
    (m2,) = axr.plot([], [], color=PALETTE["measured"], lw=3.0)
    (s2,) = axr.plot([], [], color=PALETTE["teal"], lw=1.3, ls=(0, (5, 2)))
    axr.set_xlim(0, 12.25)
    axr.set_ylim(-9, 9)
    axr.set_xlabel("time [s]")
    axr.set_ylabel("e \u2212 lag [mrad]")
    axr.set_title("Zoom into the cruise phase: the gravity ripple identifies the load", loc="left", fontsize=10)
    cruise = (t > LAB_PROFILE.t_b + 0.2) & (t < LAB_PROFILE.t_v)

    def update(i: int):
        k = min(int(round(i * STEP / 1e-3)), len(t) - 1)
        link.set_xy(bar(q[k] + show, -0.25, 0.95, 0.14))
        plates.set_xy(bar(q[k] + show, 0.62, 0.92, 0.32))
        ghost.set_xy(bar(q_d[k] + show, 0, 1.0, 0.05))
        label.set_text(f"t = {t[k]:5.2f} s\nq_d = {q_d[k]:5.2f} rad\nlag = {np.rad2deg(e_meas[k]):4.1f}\u00b0")
        m1.set_data(t[: k + 1], e_meas[: k + 1] * 1e3)
        s1.set_data(t[: k + 1], e_twin[: k + 1] * 1e3)
        sel = cruise[: k + 1]
        m2.set_data(t[: k + 1][sel], (e_meas[: k + 1][sel] - lag) * 1e3)
        s2.set_data(t[: k + 1][sel], (e_twin[: k + 1][sel] - lag) * 1e3)
        return [link, plates, ghost, label, m1, s1, m2, s2]

    n = int(round(t[-1] / STEP)) + 1
    save_gif(fig, update, range(n), "rig_twin.gif", fps=20, dpi=80, colors=96)
    update(n - 1)
    fig.savefig(MEDIA / "rig_twin.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
