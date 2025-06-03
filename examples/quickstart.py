"""Five-minute tour of the API (the output is quoted in the README)."""

from __future__ import annotations

import numpy as np
from _common import ROOT  # noqa: F401  (puts src/ on the path when the package is not installed)

from harmonictwin import (
    EXPERIMENTS,
    LAB_MODEL,
    LAB_PROFILE,
    identify_unbalance,
    load_measurements,
    model_based_voltage,
    pi_velocity_gains,
    pt2_coefficients,
    ramp_times,
    redus_gains,
    ripple_phasor,
    run_experiment,
)

print(f"M* = {LAB_MODEL.M_star:.4f} V s^2/rad, b* = {LAB_MODEL.k1:.4f} q' + {LAB_MODEL.k2:.4f} cos q  [V]")
a0, a1 = pt2_coefficients(T_R=0.08, d_R=1.0)
r0, u = model_based_voltage(LAB_MODEL, a0, a1, q_d=1.2, q=1.1, qd=1.8)
print(f"a0 = {a0:g}, a1 = {a1:g}, r0 = {r0:.3f} rad/s^2, U_S = {u:.3f} V")
print("PI velocity loop  K_P, T_N =", tuple(round(x, 6) for x in pi_velocity_gains(0.9, 0.08)))
print("ReDuS             alpha, beta, K_I =", tuple(round(x, 6) for x in redus_gains(0, 1, 0, 0.04, 0.7)))
print("ramp (prep)       t_b, t_v, t_e =", tuple(round(x, 4) for x in ramp_times(1.5, 3.0, 2 * np.pi)))

meas = load_measurements()
for runs in ((4, 5, 6), (7, 8, 9)):
    est = identify_unbalance(
        [EXPERIMENTS[k].model_mass for k in runs], [ripple_phasor(meas.t, meas.error[k], LAB_PROFILE)[0] for k in runs]
    )
    print(f"exp {runs}: unbalanced mass {est.mass:.2f} kg, gravity offset {np.rad2deg(est.offset):.2f} deg")

sim = run_experiment(7)
twin = np.interp(meas.t, sim.t, sim.error)
late = meas.t >= 0.25
print(f"exp 7 twin vs rig: RMS {np.sqrt(np.mean((twin[late] - meas.error[7][late]) ** 2)) * 1e3:.2f} mrad")
print(f"exp 7 with velocity pre-control K_V = 1: peak error {np.abs(run_experiment(7, K_V=1.0).error).max() * 1e3:.1f} mrad")
