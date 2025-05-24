"""The nine lab experiments, the identified digital twin of the rig and the measured data."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources

import numpy as np

from .controllers import CascadePI, CascadeReDuS, ModelBasedPT2
from .design import pi_velocity_gains, pt2_coefficients, redus_gains
from .params import LAB_MODEL, JointParams
from .profile import RampProfile
from .simulate import SimResult, simulate

LAB_PROFILE = RampProfile(v_m=1.5, b_m=1.0, s_e=10.0)
"""Ramp used for all experiments on the rig (the preparation used b_m = 3 and s_e = 2 pi)."""

A0, A1 = pt2_coefficients(T_R=0.08, d_R=1.0)
K_P, T_N = pi_velocity_gains(d_Rv=0.9, T_Rv=0.08)
ALPHA, BETA, K_I = redus_gains(a0=0.0, a1=1.0, a2=0.0, T_R=0.04, d_R=0.7)
K_L = 7.34  # from the PT2 approximation of the PI velocity loop; the rig used it for both cascades

RIG = JointParams(m=14.36, gravity_offset=np.deg2rad(-6.3), coulomb=0.61)
"""Digital twin of the rig with three plates mounted.

m and gravity_offset come from the gravity ripple of experiments 4-9 (see ``identify_unbalance``);
the Coulomb term explains the 2.0 V measured at 1.5 rad/s in experiment 1, where the model predicts 1.39 V.
"""


@dataclass(frozen=True)
class Experiment:
    number: int
    controller: str  # "pt2", "pi" or "redus"
    plates: bool  # three plates mounted on one side (unbalanced load)
    model_mass: float  # mass entered in the HMI model [kg]
    description: str


EXPERIMENTS = {
    1: Experiment(1, "pt2", False, 0.0, "model-based PT2, symmetric load"),
    2: Experiment(2, "pt2", True, 0.0, "model-based PT2, plates mounted, gravity not modelled"),
    3: Experiment(3, "pt2", True, 11.0, "model-based PT2, plates mounted, model mass 11 kg"),
    4: Experiment(4, "pi", True, 11.0, "P-PI cascade, model mass 11 kg"),
    5: Experiment(5, "pi", True, 16.0, "P-PI cascade, model mass 16 kg (+5 kg)"),
    6: Experiment(6, "pi", True, 0.0, "P-PI cascade, model mass 0 kg"),
    7: Experiment(7, "redus", True, 11.0, "P-ReDuS cascade, model mass 11 kg"),
    8: Experiment(8, "redus", True, 16.0, "P-ReDuS cascade, model mass 16 kg (+5 kg)"),
    9: Experiment(9, "redus", True, 0.0, "P-ReDuS cascade, model mass 0 kg"),
}


def make_controller(kind: str, model: JointParams, K_V: float = 0.0, K_A: float = 0.0, feedforward: bool = False):
    if kind == "pt2":
        return ModelBasedPT2(A0, A1, model, feedforward=feedforward)
    if kind == "pi":
        return CascadePI(K_L, K_P, T_N, model, K_V=K_V, K_A=K_A)
    if kind == "redus":
        return CascadeReDuS(K_L, ALPHA, BETA, K_I, model, K_V=K_V, K_A=K_A)
    raise ValueError(f"unknown controller {kind!r}")


def controller_for(number: int, **kwargs):
    exp = EXPERIMENTS[number]
    return make_controller(exp.controller, LAB_MODEL.with_(m=exp.model_mass), **kwargs)


def plant_for(number: int, rig: JointParams = RIG) -> JointParams:
    return rig if EXPERIMENTS[number].plates else rig.with_(m=0.0)


def run_experiment(number: int, rig: JointParams = RIG, duration: float = 12.25, **kwargs) -> SimResult:
    """Simulate lab experiment ``number`` on the digital twin (kwargs go to the controller, e.g. K_V=1)."""
    return simulate(controller_for(number, **kwargs), plant_for(number, rig), LAB_PROFILE, duration=duration)


@dataclass
class Measurements:
    t: np.ndarray
    error: dict[int, np.ndarray]


def load_measurements() -> Measurements:
    """Control deviation of experiments 4-9 recorded on the rig at 1 kHz (time as recorded)."""
    with resources.files("harmonictwin").joinpath("data/measured_control_deviation.csv").open("r", encoding="utf-8") as fh:
        lines = [line for line in fh if not line.startswith("#")]
    header = lines[0].strip().split(",")
    table = np.loadtxt(lines[1:], delimiter=",")
    error = {int(name[3:]): table[:, i] for i, name in enumerate(header) if name.startswith("exp")}
    return Measurements(t=table[:, 0], error=error)
