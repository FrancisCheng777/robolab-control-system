import numpy as np
import pytest
from pi05_demo.policy_reference import advance_joint_reference, bound_joint_reference


def test_small_displacements_survive_lag_without_runaway():
    q=np.zeros(7);ref=None
    for _ in range(100):
        ref=advance_joint_reference(q,np.ones(7)*.1,previous_reference=ref,tracking_limits=[.05]*7)
        assert np.max(abs(ref-q))<=.05+1e-12
    np.testing.assert_allclose(ref,.05)
    ref=advance_joint_reference(q,-np.ones(7)*.1,previous_reference=ref,tracking_limits=[.05]*7)
    np.testing.assert_allclose(ref,.045)  # no hidden windup delaying reversal
    np.testing.assert_allclose(advance_joint_reference(q,np.ones(7)*.1,tracking_limits=[.05]*7),.005)


def test_refresh_never_advances_reference_and_respects_tighter_limits():
    target=np.ones(7)*.05;q=np.ones(7)*.01
    for _ in range(100):target=bound_joint_reference(target,q,tracking_limits=[.02]*7)
    np.testing.assert_allclose(target,.03)


def test_perfect_tracking_matches_measured_delta_contract():
    q=np.zeros(7);ref=None
    for x in (.2,-.1,.3,0):
        expected=q+.05*x
        ref=advance_joint_reference(q,[x]*7,previous_reference=ref,tracking_limits=[.05]*7)
        np.testing.assert_allclose(ref,expected);q=ref.copy()


@pytest.mark.parametrize('limit',[0,-.1,.051,float('nan'),True])
def test_rejects_unsafe_bounds(limit):
    with pytest.raises(ValueError):bound_joint_reference([0]*7,[0]*7,tracking_limits=[.05]*7,lead_limit=limit)


def test_rejects_invalid_feedback_actions_and_limits():
    for q,a,limits in [([0]*6,[0]*7,[.05]*7),([float('nan')]*7,[0]*7,[.05]*7),([0]*7,[2]*7,[.05]*7),([0]*7,[0]*7,[0]*7)]:
        with pytest.raises(ValueError):advance_joint_reference(q,a,tracking_limits=limits)
