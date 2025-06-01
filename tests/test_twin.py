import numpy as np
import pytest

from harmonictwin import (
    EXPERIMENTS,
    LAB_MODEL,
    LAB_PROFILE,
    RIG,
    gravity_ripple_phasor,
    identify_unbalance,
    load_measurements,
    make_controller,
    ripple_phasor,
    run_experiment,
    simulate,
)

MEAS = load_measurements()


def test_measurements_are_complete():
    assert sorted(MEAS.error) == [4, 5, 6, 7, 8, 9]
    assert len(MEAS.t) == 12252
    assert np.diff(MEAS.t) == pytest.approx(1e-3)


def test_measured_cruise_error_is_the_position_loop_lag():
    for k, e in MEAS.error.items():
        _, mean_error = ripple_phasor(MEAS.t, e, LAB_PROFILE)
        assert mean_error == pytest.approx(1.5 / 7.34, abs=1e-4), k


def test_identification_recovers_a_known_plant():
    plant = LAB_MODEL.with_(m=13.0, gravity_offset=np.deg2rad(10.0))
    masses = (0.0, 11.0, 16.0)
    phasors = []
    for m in masses:
        sim = simulate(make_controller("pi", LAB_MODEL.with_(m=m)), plant, LAB_PROFILE, duration=7.0)
        phasors.append(ripple_phasor(sim.t, sim.error, LAB_PROFILE)[0])
    est = identify_unbalance(masses, phasors)
    assert est.mass == pytest.approx(13.0, abs=0.05)
    assert np.rad2deg(est.offset) == pytest.approx(10.0, abs=0.3)


@pytest.mark.parametrize("runs", [(4, 5, 6), (7, 8, 9)])
def test_both_cascades_agree_on_the_rigs_unbalance(runs):
    phasors = [ripple_phasor(MEAS.t, MEAS.error[k], LAB_PROFILE)[0] for k in runs]
    est = identify_unbalance([EXPERIMENTS[k].model_mass for k in runs], phasors)
    assert est.mass == pytest.approx(RIG.m, abs=0.15)
    assert np.rad2deg(est.offset) == pytest.approx(np.rad2deg(RIG.gravity_offset), abs=0.5)
    assert est.residual < 3e-5


@pytest.mark.parametrize("k", [4, 5, 6, 7, 8, 9])
def test_analytic_twin_predicts_the_measured_ripple(k):
    measured = ripple_phasor(MEAS.t, MEAS.error[k], LAB_PROFILE)[0]
    predicted = gravity_ripple_phasor(
        make_controller(EXPERIMENTS[k].controller, LAB_MODEL.with_(m=EXPERIMENTS[k].model_mass)), RIG, 1.5
    )
    assert abs(predicted) == pytest.approx(abs(measured), rel=0.05)
    assert abs(np.angle(predicted / measured)) < np.deg2rad(6)


@pytest.mark.parametrize("k, tolerance", [(4, 1.6e-3), (5, 1.6e-3), (6, 1.6e-3), (7, 0.6e-3), (8, 0.6e-3), (9, 0.6e-3)])
def test_simulated_twin_matches_the_recording(k, tolerance):
    sim = run_experiment(k)
    twin = np.interp(MEAS.t, sim.t, sim.error)
    late = MEAS.t >= 0.25  # the joint starts ~10 mrad off its set-point (static friction at standstill)
    assert np.sqrt(np.mean((twin[late] - MEAS.error[k][late]) ** 2)) < tolerance
