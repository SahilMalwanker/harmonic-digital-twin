"""Preparation tasks as pictures: velocity-loop step responses, PT2 approximation and the ramp profiles."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from _common import PALETTE, save

from harmonictwin import RampProfile, pi_velocity_gains, position_gain, pt2_from_step, redus_gains, step_response


def main() -> None:
    t = np.linspace(0, 1.5, 15001)
    kp, tn = pi_velocity_gains(0.9, 0.08)
    alpha, beta, ki = redus_gains(0.0, 1.0, 0.0, 0.04, 0.7)
    y_pi = step_response([tn, 1.0], [tn / kp, tn, 1.0], t)
    y_redus = step_response([beta, ki], [1.0, alpha, ki], t)
    fit = pt2_from_step(t, y_pi)
    y_fit = step_response([1.0], [fit.time_constant**2, 2 * fit.damping * fit.time_constant, 1.0], t)
    print(f"PI loop: K_P = {kp:.2f}, T_N = {tn:.3f} s, overshoot {fit.overshoot * 100:.2f} %, T_an {fit.rise_time * 1e3:.1f} ms")
    k_l = position_gain(fit.time_constant)
    print(f"         PT2 fit d = {fit.damping:.3f}, T = {fit.time_constant * 1e3:.2f} ms -> K_L = {k_l:.2f} 1/s")
    print(f"ReDuS:   alpha = {alpha:.1f}, beta = {beta:.1f}, K_I = {ki:.1f}")

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 4.2), gridspec_kw={"width_ratios": [1.15, 1]})
    ax0.plot(t * 1e3, y_pi, color=PALETTE["orange"], lw=2.2, label=f"PI velocity loop (K_P = {kp:g}, T_N = {tn:g} s)")
    ax0.plot(
        t * 1e3,
        y_fit,
        color=PALETTE["orange"],
        lw=1.2,
        ls=(0, (4, 2)),
        label=f"PT2 fit: d = {fit.damping:.2f}, T = {fit.time_constant * 1e3:.1f} ms",
    )
    ax0.plot(t * 1e3, y_redus, color=PALETTE["teal"], lw=2.2, label=f"ReDuS velocity loop (\u03b1 = {alpha:g}, K_I = {ki:g})")
    ax0.axhline(1 + fit.overshoot, color=PALETTE["muted"], lw=0.8, ls=":")
    ax0.annotate(
        f"overshoot {fit.overshoot * 100:.1f} %",
        (300, 1 + fit.overshoot),
        xytext=(0, 4),
        textcoords="offset points",
        fontsize=8.5,
    )
    ax0.axvline(fit.rise_time * 1e3, color=PALETTE["muted"], lw=0.8, ls=":")
    ax0.text(fit.rise_time * 1e3 + 5, 0.15, f"T_an = {fit.rise_time * 1e3:.1f} ms", fontsize=8.5)
    ax0.set_xlim(0, 500)
    ax0.set_ylim(-0.05, 1.3)
    ax0.set_xlabel("time [ms]")
    ax0.set_ylabel("v / v_d")
    ax0.set_title(
        f"Velocity loops on the virtual plant: K_L = 0.25 / T = {position_gain(fit.time_constant):.2f} s\u207b\u00b9", loc="left"
    )
    ax0.legend(loc="lower right")

    for prof, c, label in (
        (RampProfile(1.5, 3.0, 2 * np.pi), PALETTE["blue"], "preparation: v_m 1.5, b_m 3, s_e 2\u03c0"),
        (RampProfile(1.5, 1.0, 10.0), PALETTE["purple"], "experiments: v_m 1.5, b_m 1, s_e 10"),
    ):
        tt = np.linspace(0, 9, 2000)
        q, v, _ = prof.sample(tt)
        ax1.plot(tt, v, color=c, lw=2.0, label=f"{label}  (t_b {prof.t_b:.2f}, t_v {prof.t_v:.2f}, t_e {prof.t_e:.2f} s)")
        ax1.plot(tt, q / 5, color=c, lw=1.0, ls=(0, (4, 2)))
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("velocity [rad/s]   (dashed: angle / 5 [rad])")
    ax1.set_title("Ramp (trapezoidal velocity) profiles", loc="left")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), fontsize=8)
    fig.suptitle("Controller design from the preparation tasks", fontsize=12.5, fontweight="bold")
    fig.tight_layout()
    save(fig, "controller_design.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
