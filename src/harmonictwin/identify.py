"""Identify the rig's unbalanced load from the gravity ripple in constant-velocity experiments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .profile import RampProfile


def ripple_phasor(t, error, profile: RampProfile, window: tuple[float, float] | None = None) -> tuple[complex, float]:
    """Fit e = c0 + Re(P e^{iq}) on the cruise phase; returns (P, c0).

    q is reconstructed as q_d(t) - e(t). Any once-per-revolution disturbance (gravity) shows up in P.
    """
    t = np.asarray(t, dtype=float)
    e = np.asarray(error, dtype=float)
    lo, hi = window if window is not None else (profile.t_b + 1.0, profile.t_v - 0.2)
    m = (t >= lo) & (t <= hi)
    q = profile.sample(t[m])[0] - e[m]
    A = np.column_stack([np.ones(m.sum()), np.cos(q), np.sin(q)])
    (c0, cc, cs), *_ = np.linalg.lstsq(A, e[m], rcond=None)
    return complex(cc, -cs), float(c0)


@dataclass
class UnbalanceEstimate:
    mass: float  # true unbalanced mass (at the model's l_s) [kg]
    offset: float  # angle offset of the true gravity term relative to the model's cos(q) [rad]
    gain: complex  # closed-loop factor G in P = G (m e^{i offset} - m_model)
    residual: float  # largest misfit of the straight-line model [rad]


def identify_unbalance(model_masses, phasors) -> UnbalanceEstimate:
    """Solve P_k = G (m e^{i phi} - m_k) for (m, phi, G) from runs that differ only in the model mass m_k.

    A gravity mismatch enters the loop linearly, so the ripple phasor is a straight line in m_k.
    Where that line passes closest to zero, the model would cancel gravity exactly.
    """
    mk = np.asarray(model_masses, dtype=float)
    P = np.asarray(phasors, dtype=complex)
    X = np.column_stack([np.ones_like(mk), mk]).astype(complex)
    (a, b), *_ = np.linalg.lstsq(X, P, rcond=None)
    w = -a / b
    residual = float(np.abs(P - X @ np.array([a, b])).max())
    return UnbalanceEstimate(mass=float(abs(w)), offset=float(np.angle(w)), gain=complex(-b), residual=residual)


def friction_from_cruise(u_cruise: float, velocity: float, model_k1: float) -> tuple[float, float]:
    """Split a cruise-phase voltage into the modelled viscous part and the unexplained rest (task 5.1).

    Returns (effective k1 [V s/rad], extra voltage [V]).
    """
    return u_cruise / velocity, u_cruise - model_k1 * velocity
