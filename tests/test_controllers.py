import numpy as np
import pytest

from harmonictwin import (
    LAB_MODEL,
    LAB_PROFILE,
    CascadePI,
    CascadeReDuS,
    ModelBasedPT2,
    disturbance_response,
    ramp_lag,
    simulate,
)

PLANT = LAB_MODEL.with_(m=0.0)


def cruise(sim):
    return sim.error[(sim.t > 3.0) & (sim.t < 6.5)]


@pytest.mark.parametrize(
    "controller",
    [
        ModelBasedPT2(156.25, 25.0, PLANT),
        CascadePI(7.34, 22.5, 0.144, PLANT),
        CascadeReDuS(7.34, 35.0, 0.0, 625.0, PLANT),
    ],
)
def test_cruise_error_equals_the_predicted_ramp_lag(controller):
    sim = simulate(controller, PLANT, LAB_PROFILE, duration=7.0)
    assert cruise(sim).mean() == pytest.approx(ramp_lag(controller, 1.5), abs=2e-4)


def test_cascades_lag_by_velocity_over_position_gain():
    assert ramp_lag(CascadePI(7.34, 22.5, 0.144, PLANT), 1.5) == pytest.approx(1.5 / 7.34)
    assert ramp_lag(CascadePI(7.34, 22.5, 0.144, PLANT, K_V=1.0), 1.5) == 0.0


def test_computed_torque_with_a_perfect_model_tracks_exactly():
    sim = simulate(ModelBasedPT2(156.25, 25.0, PLANT, feedforward=True), PLANT, LAB_PROFILE)
    assert np.abs(sim.error).max() < 2e-4


def test_integral_action_removes_steady_state_error_from_unmodelled_gravity():
    plant = LAB_MODEL.with_(m=12.0)
    for controller in (CascadePI(7.34, 22.5, 0.144, PLANT), CascadeReDuS(7.34, 35.0, 0.0, 625.0, PLANT)):
        sim = simulate(controller, plant, LAB_PROFILE)
        assert abs(sim.error[sim.t > 11.5].mean()) < 2e-4
    sim = simulate(ModelBasedPT2(156.25, 25.0, PLANT), plant, LAB_PROFILE)
    assert abs(sim.error[sim.t > 11.5].mean()) > 5e-3


def test_disturbance_response_limits():
    pt2 = ModelBasedPT2(156.25, 25.0, PLANT)
    assert disturbance_response(pt2, 1e-9) == pytest.approx(1 / 156.25)
    pi = CascadePI(7.34, 22.5, 0.144, PLANT)
    assert abs(disturbance_response(pi, 1e-9)) < 1e-9
    assert abs(disturbance_response(CascadeReDuS(7.34, 35.0, 0.0, 625.0, PLANT), 1.5)) < abs(disturbance_response(pi, 1.5))


def test_voltage_limit_is_respected():
    sim = simulate(CascadePI(7.34, 22.5, 0.144, PLANT), PLANT, LAB_PROFILE, duration=2.0, voltage_limit=1.0)
    assert np.abs(sim.u).max() <= 1.0
