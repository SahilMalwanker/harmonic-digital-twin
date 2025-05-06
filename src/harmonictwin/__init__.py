"""harmonictwin: digital twin of a harmonic-drive robot joint under model-based control, validated on lab measurements."""

from .controllers import (
    CascadePI,
    CascadeReDuS,
    ModelBasedPT2,
    Reference,
    disturbance_response,
    gravity_ripple_phasor,
    ramp_lag,
    ripple_gain,
)
from .design import (
    PT2Fit,
    model_based_voltage,
    pi_velocity_gains,
    position_gain,
    pt2_coefficients,
    pt2_from_step,
    redus_gains,
    step_response,
)
from .experiments import (
    EXPERIMENTS,
    LAB_PROFILE,
    RIG,
    Experiment,
    controller_for,
    load_measurements,
    make_controller,
    plant_for,
    run_experiment,
)
from .identify import UnbalanceEstimate, friction_from_cruise, identify_unbalance, ripple_phasor
from .params import LAB_MODEL, JointParams
from .profile import RampProfile, ramp_times
from .simulate import SimResult, simulate

__version__ = "1.0.0"

__all__ = [
    "EXPERIMENTS",
    "LAB_MODEL",
    "LAB_PROFILE",
    "RIG",
    "CascadePI",
    "CascadeReDuS",
    "Experiment",
    "JointParams",
    "ModelBasedPT2",
    "PT2Fit",
    "RampProfile",
    "Reference",
    "SimResult",
    "UnbalanceEstimate",
    "controller_for",
    "disturbance_response",
    "friction_from_cruise",
    "gravity_ripple_phasor",
    "identify_unbalance",
    "load_measurements",
    "make_controller",
    "model_based_voltage",
    "pi_velocity_gains",
    "plant_for",
    "position_gain",
    "pt2_coefficients",
    "pt2_from_step",
    "ramp_lag",
    "ramp_times",
    "redus_gains",
    "ripple_gain",
    "ripple_phasor",
    "run_experiment",
    "simulate",
    "step_response",
]
