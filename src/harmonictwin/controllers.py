"""The rig's controllers: pure model-based PT2 control and the P-PI / P-ReDuS cascades.

Every controller computes the virtual-plant input r0 (an acceleration) and maps it through the
inverse model of its *believed* parameters: U_S = M*~ r0 + b*~(q, q').
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .params import JointParams


@dataclass
class Reference:
    q: float
    v: float
    a: float


@dataclass
class ModelBasedPT2:
    """r0 = a0 (q_d - q) - a1 q' (lab eq. 13b). With ``feedforward`` it becomes full computed-torque control."""

    a0: float
    a1: float
    model: JointParams
    feedforward: bool = False
    name: str = "model-based PT2"

    def reset(self) -> None:
        pass

    def __call__(self, ref: Reference, q: float, qd: float, dt: float) -> float:
        r0 = self.a0 * (ref.q - q) - self.a1 * qd
        if self.feedforward:
            r0 += ref.a + self.a1 * ref.v
        return self.model.M_star * r0 + self.model.b_star(q, qd)


@dataclass
class CascadePI:
    """P position loop around a PI velocity loop on the compensated (double-integrator) plant."""

    K_L: float
    K_P: float
    T_N: float
    model: JointParams
    K_V: float = 0.0  # velocity pre-control, 0..1
    K_A: float = 0.0  # acceleration feed-forward, 0..1
    name: str = "P-PI cascade"
    _integral: float = field(default=0.0, init=False, repr=False)

    def reset(self) -> None:
        self._integral = 0.0

    def __call__(self, ref: Reference, q: float, qd: float, dt: float) -> float:
        v_set = self.K_L * (ref.q - q) + self.K_V * ref.v
        ev = v_set - qd
        self._integral += ev * dt
        r0 = self.K_P * (ev + self._integral / self.T_N) + self.K_A * ref.a
        return self.model.M_star * r0 + self.model.b_star(q, qd)


@dataclass
class CascadeReDuS:
    """P position loop around a ReDuS velocity loop: r0 = K_I * integral(v_set - q') + beta v_set - alpha q'."""

    K_L: float
    alpha: float
    beta: float
    K_I: float
    model: JointParams
    K_V: float = 0.0
    K_A: float = 0.0
    name: str = "P-ReDuS cascade"
    _integral: float = field(default=0.0, init=False, repr=False)

    def reset(self) -> None:
        self._integral = 0.0

    def __call__(self, ref: Reference, q: float, qd: float, dt: float) -> float:
        v_set = self.K_L * (ref.q - q) + self.K_V * ref.v
        self._integral += (v_set - qd) * dt
        # Feed-forward also cancels the -alpha q' state feedback, so perfect tracking needs no integrator state.
        ff = self.K_A * (ref.a + (self.alpha - self.beta) * ref.v)
        r0 = self.K_I * self._integral + self.beta * v_set - self.alpha * qd + ff
        return self.model.M_star * r0 + self.model.b_star(q, qd)


def disturbance_response(controller, omega: float) -> complex:
    """H(i omega) from an acceleration disturbance d to the position error e = q_d - q.

    The plant is the ideally compensated double integrator q'' = r0 + d, so H only depends on the
    controller gains. Error phasor = -H * disturbance phasor.
    """
    s = 1j * omega
    if isinstance(controller, ModelBasedPT2):
        return 1.0 / (s**2 + controller.a1 * s + controller.a0)
    if isinstance(controller, CascadePI):
        k_i = controller.K_P / controller.T_N
        den = s**3 + controller.K_P * s**2 + (controller.K_P * controller.K_L + k_i) * s + k_i * controller.K_L
        return s / den
    if isinstance(controller, CascadeReDuS):
        c = controller
        den = s**3 + c.alpha * s**2 + (c.K_I + c.beta * c.K_L) * s + c.K_I * c.K_L
        return s / den
    raise TypeError(f"unsupported controller {type(controller).__name__}")


def ramp_lag(controller, velocity: float) -> float:
    """Steady position error while following a constant-velocity reference (no disturbance)."""
    if isinstance(controller, ModelBasedPT2):
        return 0.0 if controller.feedforward else controller.a1 * velocity / controller.a0
    return (1.0 - controller.K_V) * velocity / controller.K_L


def ripple_gain(controller, true: JointParams, velocity: float) -> complex:
    """Error phasor per kilogram of uncompensated unbalanced mass while cruising at ``velocity``."""
    per_kg = true.g * true.l_s / (true.K_M * true.u * true.M_star)
    return disturbance_response(controller, velocity) * per_kg


def gravity_ripple_phasor(controller, true: JointParams, velocity: float) -> complex:
    """Predicted error phasor P (e = Re(P e^{iq})) caused by a gravity-model mismatch at constant velocity."""
    mismatch = true.m * np.exp(1j * true.gravity_offset) - controller.model.m * np.exp(1j * controller.model.gravity_offset)
    return ripple_gain(controller, true, velocity) * mismatch
