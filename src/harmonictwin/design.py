"""Controller design formulas from the preparation tasks, plus a small LTI step-response helper."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .params import JointParams


def pt2_coefficients(T_R: float, d_R: float) -> tuple[float, float]:
    """a0 = 1/T_R^2, a1 = 2 d_R / T_R for the desired PT2 behaviour q/q_d (eq. 12-13)."""
    return 1.0 / T_R**2, 2.0 * d_R / T_R


def model_based_voltage(model: JointParams, a0: float, a1: float, q_d: float, q: float, qd: float) -> tuple[float, float]:
    """(r0, U_S) of the pure model-based controller at one instant (task 4.2)."""
    r0 = a0 * (q_d - q) - a1 * qd
    return r0, float(model.M_star * r0 + model.b_star(q, qd))


def pi_velocity_gains(d_Rv: float, T_Rv: float) -> tuple[float, float]:
    """(K_P, T_N) that give the closed velocity loop the denominator of a PT2 (eq. 20)."""
    T_N = 2.0 * d_Rv * T_Rv
    return T_N / T_Rv**2, T_N


def redus_gains(a0: float, a1: float, a2: float, T_R: float, d_R: float) -> tuple[float, float, float]:
    """(alpha, beta, K_I) of the ReDuS velocity controller for the plant 1/(a0 + a1 s + a2 s^2) (eq. 27)."""
    if T_R * a1 - 2.0 * d_R * a2 <= 0.0:
        raise ValueError("T_R a1 - 2 d_R a2 must be positive (eq. 28)")
    beta = a2 / T_R**2
    K_I = (a1 - 2.0 * d_R * T_R * beta) / T_R**2
    alpha = 2.0 * d_R * T_R * K_I + beta - a0
    return alpha, beta, K_I


def position_gain(T_v: float, factor: float = 0.25) -> float:
    """Rule of thumb 0.2/T_v <= K_L <= 0.3/T_v (eq. 29); ``factor`` picks the point in that range."""
    if not 0.2 <= factor <= 0.3:
        raise ValueError("factor should lie in [0.2, 0.3]")
    return factor / T_v


def _expm(M: np.ndarray) -> np.ndarray:
    """Matrix exponential by scaling and squaring of a Taylor series (fine for the small matrices here)."""
    norm = np.abs(M).sum(axis=1).max()
    s = max(0, int(np.ceil(np.log2(norm))) + 1) if norm > 0 else 0
    X = M / 2.0**s
    E = np.eye(len(M))
    term = np.eye(len(M))
    for k in range(1, 18):
        term = term @ X / k
        E = E + term
    for _ in range(s):
        E = E @ E
    return E


def step_response(num, den, t: np.ndarray) -> np.ndarray:
    """Exact unit-step response of num(s)/den(s) (polynomials, highest power first) at the times t."""
    num = np.atleast_1d(np.asarray(num, dtype=float))
    den = np.atleast_1d(np.asarray(den, dtype=float))
    num, den = num / den[0], den / den[0]
    n = len(den) - 1
    num = np.r_[np.zeros(n + 1 - len(num)), num]
    A = np.zeros((n, n))
    A[:-1, 1:] = np.eye(n - 1)
    A[-1] = -den[:0:-1]
    B = np.zeros(n)
    B[-1] = 1.0
    Cv = num[:0:-1] - num[0] * den[:0:-1]
    D = num[0]
    t = np.asarray(t, dtype=float)
    y = np.empty(len(t))
    x = np.zeros(n)
    y[0] = Cv @ x + D
    cache: dict[float, tuple[np.ndarray, np.ndarray]] = {}
    for k in range(1, len(t)):
        h = round(float(t[k] - t[k - 1]), 15)
        if h not in cache:
            aug = np.zeros((n + 1, n + 1))
            aug[:n, :n] = A * h
            aug[:n, n] = B * h
            E = _expm(aug)
            cache[h] = (E[:n, :n], E[:n, n])
        phi, gamma = cache[h]
        x = phi @ x + gamma
        y[k] = Cv @ x + D
    return y


@dataclass
class PT2Fit:
    overshoot: float  # (v_max - v_inf) / v_inf
    damping: float  # d_v
    rise_time: float  # T_an, 0 -> 100 % [s]
    time_constant: float  # T_v [s]


def pt2_from_step(t: np.ndarray, y: np.ndarray) -> PT2Fit:
    """Approximate a measured step response by a PT2 from overshoot and 0-100 % rise time (eq. 21-23)."""
    y_inf = float(y[-1])
    overshoot = (float(np.max(y)) - y_inf) / y_inf
    damping = 1.0 / np.sqrt(1.0 + (np.pi / np.log(overshoot)) ** 2)
    k = int(np.argmax(y >= y_inf))
    # Interpolate the first crossing of the final value for a resolution-independent rise time.
    rise = float(t[k - 1] + (y_inf - y[k - 1]) * (t[k] - t[k - 1]) / (y[k] - y[k - 1])) if k > 0 else float(t[0])
    T_v = rise * np.sqrt(1.0 - damping**2) / (np.pi - np.arccos(damping))
    return PT2Fit(overshoot=overshoot, damping=float(damping), rise_time=rise, time_constant=float(T_v))
