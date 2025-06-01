import numpy as np
import pytest

from harmonictwin import (
    LAB_MODEL,
    RampProfile,
    model_based_voltage,
    pi_velocity_gains,
    position_gain,
    pt2_coefficients,
    pt2_from_step,
    ramp_times,
    redus_gains,
    step_response,
)


def test_model_parameters_of_task_4_1():
    assert LAB_MODEL.K_M == pytest.approx(0.22 / 0.8475)
    assert LAB_MODEL.M_star == pytest.approx(0.3377, abs=1e-4)
    assert LAB_MODEL.k1 == pytest.approx(0.9245, abs=1e-4)
    assert LAB_MODEL.k2 == pytest.approx(0.8881, abs=1e-4)


def test_inverse_model_is_consistent():
    q, qd, qdd = 0.3, 1.2, -0.7
    expected = LAB_MODEL.M_star * qdd + LAB_MODEL.k1 * qd + LAB_MODEL.k2 * np.cos(q)
    assert LAB_MODEL.inverse_model(q, qd, qdd) == pytest.approx(expected)


def test_pt2_model_based_control_of_task_4_2():
    a0, a1 = pt2_coefficients(0.08, 1.0)
    assert (a0, a1) == pytest.approx((156.25, 25.0))
    r0, u = model_based_voltage(LAB_MODEL, a0, a1, q_d=1.2, q=1.1, qd=1.8)
    assert r0 == pytest.approx(-29.375)
    assert u == pytest.approx(-7.854, abs=1e-3)


def test_pi_velocity_controller_of_task_4_3():
    assert pi_velocity_gains(0.9, 0.08) == pytest.approx((22.5, 0.144))


def test_step_response_matches_the_closed_form():
    kp, tn = 22.5, 0.144
    ki = kp / tn
    t = np.linspace(0, 0.6, 601)
    y = step_response([tn, 1.0], [tn / kp, tn, 1.0], t)
    wn = np.sqrt(ki)
    sig = kp / 2
    wd = np.sqrt(wn**2 - sig**2)
    exact = 1 - np.exp(-sig * t) * (np.cos(wd * t) + (sig - kp) / wd * np.sin(wd * t))
    np.testing.assert_allclose(y, exact, atol=1e-8)


def test_pt2_fit_recovers_a_pure_pt2():
    d, T = 0.5, 0.05
    t = np.linspace(0, 1, 100001)
    y = step_response([1.0], [T**2, 2 * d * T, 1.0], t)
    fit = pt2_from_step(t, y)
    assert fit.damping == pytest.approx(d, abs=1e-3)
    assert fit.time_constant == pytest.approx(T, rel=1e-3)


def test_pi_loop_step_and_position_gain():
    kp, tn = 22.5, 0.144
    t = np.linspace(0, 1, 20001)
    fit = pt2_from_step(t, step_response([tn, 1.0], [tn / kp, tn, 1.0], t))
    assert fit.overshoot == pytest.approx(0.1553, abs=5e-4)
    assert fit.rise_time == pytest.approx(0.0828, abs=5e-4)
    assert position_gain(fit.time_constant) == pytest.approx(7.39, abs=0.01)
    with pytest.raises(ValueError):
        position_gain(0.03, factor=0.5)


def test_redus_parameters_of_task_5_3():
    assert redus_gains(0.0, 1.0, 0.0, 0.04, 0.7) == pytest.approx((35.0, 0.0, 625.0))
    with pytest.raises(ValueError):
        redus_gains(0.0, 1.0, 1.0, 0.04, 0.7)


def test_ramp_times_of_task_4_4_and_of_the_experiments():
    assert ramp_times(1.5, 3.0, 2 * np.pi) == pytest.approx((0.5, 2 * np.pi / 1.5, 2 * np.pi / 1.5 + 0.5))
    assert ramp_times(1.5, 1.0, 10.0) == pytest.approx((1.5, 10 / 1.5, 10 / 1.5 + 1.5))


@pytest.mark.parametrize("profile", [RampProfile(), RampProfile(v_m=2.0, b_m=1.0, s_e=1.0, q0=0.5, t0=0.3)])
def test_profile_is_consistent(profile):
    t = np.linspace(-0.5, profile.t0 + profile.t_e + 1.0, 200001)
    q, v, a = profile.sample(t)
    dt = t[1] - t[0]
    np.testing.assert_allclose(np.diff(q) / dt, 0.5 * (v[1:] + v[:-1]), atol=1e-3)
    np.testing.assert_allclose(np.diff(v) / dt, 0.5 * (a[1:] + a[:-1]), atol=2.0)
    assert q[-1] == pytest.approx(profile.q0 + profile.s_e)
    assert v.max() == pytest.approx(profile.peak_velocity, rel=1e-4)
