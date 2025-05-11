"""Trapezoidal (ramp) velocity profile used for every motion of the rig."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RampProfile:
    """Accelerate with b_m, cruise at v_m, decelerate with b_m; travel s_e from q0 starting at t0."""

    v_m: float = 1.5
    b_m: float = 1.0
    s_e: float = 10.0
    q0: float = 0.0
    t0: float = 0.0

    @property
    def peak_velocity(self) -> float:
        # Short moves never reach v_m: the profile degenerates to a triangle.
        return min(self.v_m, float(np.sqrt(self.s_e * self.b_m)))

    @property
    def t_b(self) -> float:
        """End of the acceleration phase."""
        return self.peak_velocity / self.b_m

    @property
    def t_e(self) -> float:
        """End of the motion."""
        return self.s_e / self.peak_velocity + self.t_b

    @property
    def t_v(self) -> float:
        """Start of the deceleration phase."""
        return self.t_e - self.t_b

    def sample(self, t):
        """(position, velocity, acceleration) at time(s) t."""
        t = np.asarray(t, dtype=float) - self.t0
        b, v, tb, tv, te = self.b_m, self.peak_velocity, self.t_b, self.t_v, self.t_e
        accel = (t >= 0) & (t < tb)
        cruise = (t >= tb) & (t < tv)
        decel = (t >= tv) & (t < te)
        done = t >= te
        a = np.where(accel, b, np.where(decel, -b, 0.0))
        vel = np.where(accel, b * t, np.where(cruise, v, np.where(decel, b * (te - t), 0.0)))
        pos = np.where(
            accel,
            0.5 * b * t**2,
            np.where(
                cruise, v * (t - 0.5 * tb), np.where(decel, self.s_e - 0.5 * b * (te - t) ** 2, np.where(done, self.s_e, 0.0))
            ),
        )
        return self.q0 + pos, vel, a


def ramp_times(v_m: float, b_m: float, s_e: float) -> tuple[float, float, float]:
    """(t_b, t_v, t_e) of the lab's ramp profile (task 4.4)."""
    p = RampProfile(v_m=v_m, b_m=b_m, s_e=s_e)
    return p.t_b, p.t_v, p.t_e
