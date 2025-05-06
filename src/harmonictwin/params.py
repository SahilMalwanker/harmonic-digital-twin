"""Parameters of the single-joint test rig and the lab's rigid-body model."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np


@dataclass(frozen=True)
class JointParams:
    """Joint 3 of a 6-axis arm on a test stand: brushless motor, harmonic drive and an unbalanced link.

    The equation of motion in control-voltage units is M* q'' = U_S - b*(q, q') with
    M* = (M + J_A u^2) / (K_M u) and b* = k1 q' + k2 cos(q + gravity_offset) + U_c sign(q').
    """

    u: float = 160.0  # gear ratio
    C: float = 0.22  # motor constant [N m / A]
    J_A: float = 3.37e-4  # motor-side inertia [kg m^2]
    K_MI: float = 0.8475  # current controller: U_S = K_MI * I_A [V / A]
    F_M: float = 0.0015  # viscous friction, motor side [N m s / rad]
    M: float = 5.4  # link inertia, joint side [kg m^2]
    m: float = 9.4  # unbalanced mass [kg]
    l_s: float = 0.4  # distance from the axis to its centre of mass [m]
    g: float = 9.81
    gravity_offset: float = 0.0  # angle between q = 0 and the horizontal position of the load [rad]
    coulomb: float = 0.0  # Coulomb friction in control-voltage units [V]

    @property
    def K_M(self) -> float:
        return self.C / self.K_MI

    @property
    def M_star(self) -> float:
        return (self.M + self.J_A * self.u**2) / (self.K_M * self.u)

    @property
    def k1(self) -> float:
        """Viscous term of b* [V s / rad]."""
        return self.F_M * self.u**2 / (self.K_M * self.u)

    @property
    def k2(self) -> float:
        """Gravity term of b* [V]."""
        return self.m * self.g * self.l_s / (self.K_M * self.u)

    def b_star(self, q, qd, coulomb_width: float = 0.02):
        """Velocity- and position-dependent part of the inverse model, in volts."""
        friction = self.coulomb * np.tanh(np.asarray(qd) / coulomb_width) if self.coulomb else 0.0
        return self.k1 * qd + self.k2 * np.cos(q + self.gravity_offset) + friction

    def inverse_model(self, q, qd, qdd):
        """U_S = M* q'' + b*(q, q') (lab eq. 11)."""
        return self.M_star * qdd + self.b_star(q, qd)

    def with_(self, **changes) -> JointParams:
        return replace(self, **changes)


LAB_MODEL = JointParams()
"""The handout's parameters (three plates on one side, m = 9.4 kg)."""
