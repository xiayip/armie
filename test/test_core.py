import json
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from armie.hmi import display_packet
from armie.motion import Motion, gesture_points
from armie.perception import mock_scene, validate_scene


@pytest.mark.parametrize('pose', [[], [0.] * 5, [float('nan')] * 6,
                                      [0., -1., 0., 0., 0., 0.]])
def test_invalid_pose_never_builds_trajectory(pose):
    with pytest.raises(ValueError):
        gesture_points(pose, 'nod')


@pytest.mark.parametrize('gesture', ['greet', 'nod', 'tilt'])
def test_gesture_returns_to_measured_pose_and_respects_peak_speed(gesture):
    pose = [0., .6, .8, .2, .1, 0.]
    points = gesture_points(pose, gesture)
    assert points[0][1] == points[-1][1] == pose
    for (t0, q0), (t1, q1) in zip(points, points[1:]):
        assert t1 > t0
        assert max(1.875 * abs(b-a) / (t1-t0) for a, b in zip(q0, q1)) <= .12
    assert pose == [0., .6, .8, .2, .1, 0.]


@pytest.mark.parametrize('update', [dict(confidence=float('nan')), dict(confidence=True),
                                    dict(person_present='yes'), dict(mood='move_arm'),
                                    dict(joint_targets=[1.] * 6), dict(summary='x' * 241)])
def test_reject_invalid_or_action_bearing_vlm_output(update):
    with pytest.raises(ValueError):
        validate_scene(dict(mock_scene(), **update))


def test_display_protocol_is_bounded_and_disjoint_from_controller_protocol():
    packet = display_packet(dict(schema_version=1, sequence=2, expression='happy',
                                 text='字' * 1000, ttl_ms=2500))
    assert len(packet['text']) == 240
    assert packet['protocol'] == 'armie-face'
    assert 'mode' not in packet and 'token' not in packet
    assert len(json.dumps(packet, ensure_ascii=False).encode()) < 1200


def test_stop_during_goal_acceptance_cancels_late_owned_goal():
    events, cancels = [], []
    motion = Motion.__new__(Motion)
    motion.busy, motion.phase, motion.handle = True, 'sending', None
    motion.backend, motion.cancel_requested, motion.cancel_sent = 'trajectory', False, False
    motion.report = lambda *args: events.append(args)
    result, cancelled = Future(), Future()
    cancelled.set_result(SimpleNamespace(goals_canceling=[1]))
    def cancel():
        cancels.append('owned')
        return cancelled
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                             cancel_goal_async=cancel)
    motion.stop()
    assert motion.busy
    accepted = Future()
    accepted.set_result(handle)
    motion.goal_done(accepted)
    assert cancels == ['owned'] and motion.busy
    assert motion.start('nod')[0] is False
    result.set_result(SimpleNamespace(status=5, result=SimpleNamespace(error_code=0)))
    assert not motion.busy and events[-1][0] is False


def test_unknown_goal_acceptance_keeps_gate_closed():
    motion = Motion.__new__(Motion)
    motion.busy = True
    motion.report = lambda *args: None
    future = Future()
    future.set_exception(RuntimeError('transport lost'))
    motion.goal_done(future)
    assert motion.phase == 'uncertain' and motion.start('nod')[0] is False
