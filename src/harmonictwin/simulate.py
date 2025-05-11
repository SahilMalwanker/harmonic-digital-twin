"""Closed-loop simulation: 1 kHz digital controller, voltage limit, RK4 plant integration."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .controllers import Reference
from .params import JointParams
from .profile import RampProfile


@dataclass
class SimResult:
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    q_ref: np.ndarray
    v_ref: np.ndarray
    a_ref: np.ndarray
    u: np.ndarray  # control voltage U_S [V]

    @property
    def error(self) -> np.ndarray:
        """Control deviation e = q_d - q [rad]."""
        return self.q_ref - self.q


def simulate(
    controller,
    plant: JointParams,
    profile: RampProfile | None = None,
    duration: float = 12.25,
    dt: float = 1e-3,
    substeps: int = 4,
    voltage_limit: float = 10.0,
    q0: float = 0.0,
) -> SimResult:
    """Run ``controller`` against the ``plant`` along ``profile``.

    The controller samples position every ``dt`` (T_A = 1 ms on the rig) and differentiates it for the
    velocity, like the drive's incremental sensor. The plant is integrated with RK4 in ``substeps`` steps.
    """
    profile = profile or RampProfile()
    controller.reset()
    n = int(round(duration / dt)) + 1
    t = np.arange(n) * dt
    q_ref, v_ref, a_ref = profile.sample(t)
    q_ref = q_ref + q0
    out_q, out_qd, out_u = np.empty(n), np.empty(n), np.empty(n)
    q, qd = q0, 0.0
    q_prev = q0
    h = dt / substeps
    k1_, k2_, offset, coulomb = plant.k1, plant.k2, plant.gravity_offset, plant.coulomb
    inv_m = 1.0 / plant.M_star

    def accel(qq, vv, u):
        friction = coulomb * math.tanh(vv / 0.02) if coulomb else 0.0
        return (u - k1_ * vv - k2_ * math.cos(qq + offset) - friction) * inv_m

    for k in range(n):
        v_meas = (q - q_prev) / dt if k else 0.0
        u = controller(Reference(q_ref[k], v_ref[k], a_ref[k]), q, v_meas, dt)
        u = float(np.clip(u, -voltage_limit, voltage_limit))
        out_q[k], out_qd[k], out_u[k] = q, qd, u
        q_prev = q
        for _ in range(substeps):
            k1q, k1v = qd, accel(q, qd, u)
            k2q, k2v = qd + 0.5 * h * k1v, accel(q + 0.5 * h * k1q, qd + 0.5 * h * k1v, u)
            k3q, k3v = qd + 0.5 * h * k2v, accel(q + 0.5 * h * k2q, qd + 0.5 * h * k2v, u)
            k4q, k4v = qd + h * k3v, accel(q + h * k3q, qd + h * k3v, u)
            q += h / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q)
            qd += h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return SimResult(t=t, q=out_q, qd=out_qd, q_ref=q_ref, v_ref=v_ref, a_ref=a_ref, u=out_u)
