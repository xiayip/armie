import time
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from armie.ball_sequence import timed_points, JOINTS
from armie.ball_grasp_geometry import grasp_evidence, preview_waypoints, wrapup_waypoints
from armie.game import Game
from armie.senses import Senses
from test_ros import graph, until


def trajectory():
    t=JointTrajectory(joint_names=JOINTS)
    for sec,q in [(0,[0.,.6,.8,0.,0.,0.]),(1,[0.,.62,.8,0.,0.,0.])]:
        p=JointTrajectoryPoint(positions=q,velocities=[0.]*6,accelerations=[0.]*6)
        p.time_from_start.sec=sec;t.points.append(p)
    return t


def test_cartesian_samples_preserved_and_invalid_paths_rejected():
    t=trajectory();points=timed_points(t,t.points[0].positions)
    assert [list(p.positions) for p in points]==[list(p.positions) for p in t.points]
    t.points[1].positions[1]=0.
    with pytest.raises(ValueError,match='limit'):timed_points(t,t.points[0].positions)
    t=trajectory();t.points[-1].velocities[0]=.1
    with pytest.raises(ValueError,match='stopped'):timed_points(t,t.points[0].positions)
    t=trajectory();t.points[1].time_from_start.sec=0
    with pytest.raises(ValueError,match='duration'):timed_points(t,t.points[0].positions)


PARAMS=['mock_step_sec:=0.03','ball_full_sequence:=true','ball_still_sec:=0.5',
        'grasp_enabled:=true','gripper_open_only:=false','ball_controller:=position']


def test_full_mock_sequence_closes_only_after_grasp_then_places_and_returns_home():
    with graph([Game,Senses],PARAMS) as (nodes,exe):
        game,senses,probe=nodes;stages=[];events=[]
        original=game.set_state
        def state(s,p,m):stages.append(p);original(s,p,m)
        game.set_state=state
        grip=game.robot.grip
        def record(closed,cb):events.append((closed,game.phase));return grip(closed,cb)
        game.robot.grip=record
        assert game.command('2')[0]
        until(exe,lambda:'grasp_lift' in stages and game.state=='idle',timeout=20)
        assert [p for p in stages if p.startswith('grasp_')]==[
            'grasp_settling','grasp_approach','grasp_look_left','grasp_look_right','grasp_pregrasp','grasp_grasp',
            'grasp_closing','grasp_lift','grasp_verify','grasp_show','grasp_place','grasp_placing',
            'grasp_clear','grasp_home','grasp_wrapup']
        assert [phase for closed,phase in events if closed]==['grasp_closing']
        assert events[-1]==(False,'grasp_placing') and not game.robot.grip_closed
        assert 'puzzled' not in stages and stages[-2:]==['returning','waiting']
        assert game.outcome=='simulated_grasp' and game.mode=='' and not game.ball_sequence.busy


def test_mock_missed_grasp_opens_retreats_and_tilts_without_placing():
    with graph([Game,Senses],PARAMS+['mock_grasp_success:=false']) as (nodes,exe):
        game,senses,probe=nodes;stages=[]
        original=game.set_state
        def state(s,p,m):stages.append(p);original(s,p,m)
        game.set_state=state
        assert game.command('2')[0]
        until(exe,lambda:'grasp_lift' in stages and game.state=='idle',timeout=20)
        after=stages[stages.index('grasp_verify'):]
        assert [p for p in after if p.startswith('grasp_') or p=='puzzled']==[
            'grasp_verify','grasp_missed','grasp_home','grasp_wrapup','puzzled']
        assert game.outcome=='grasp_missed' and not game.robot.grip_closed


def test_grasp_evidence_requires_gripper_and_camera_agreement():
    assert grasp_evidence(.55,5,False)[0]
    assert not grasp_evidence(None,5,False)[0]
    assert '完全闭合' in grasp_evidence(.97,5,False)[1]
    assert not grasp_evidence(.02,5,False)[0]
    assert '观测不足' in grasp_evidence(.55,1,False)[1]
    assert '桌面原位' in grasp_evidence(.55,5,True)[1]


def test_wrapup_places_slightly_above_pickup_then_returns_to_start():
    waypoints=preview_waypoints([.3,0.,.1],.025)
    home=dict(stage='start',position=[.3,0.,.3],orientation=[1.,0.,0.,0.])
    place,clear,back=wrapup_waypoints(waypoints,home,True)
    assert [place['stage'],clear['stage'],back['stage']]==['place','clear','home']
    assert place['position']==pytest.approx([.3,0.,.11]) and back['position']==home['position']
    assert clear['position']==waypoints[3]['position']
    assert [w['stage'] for w in wrapup_waypoints(waypoints,home,False)]==['home']


def test_finish_mid_grasp_cancels_sequence_opens_and_returns_home():
    with graph([Game,Senses],PARAMS+['mock_step_sec:=0.15']) as (nodes,exe):
        game,senses,probe=nodes;stages=[]
        original=game.set_state
        def state(s,p,m):stages.append(p);original(s,p,m)
        game.set_state=state
        assert game.command('2')[0]
        until(exe,lambda:game.phase=='grasp_lift',timeout=20)
        assert game.robot.grip_closed
        assert game.command('finish')[0]
        assert not game.command('finish')[0]
        until(exe,lambda:game.state=='idle',timeout=12)
        assert not game.ball_sequence.busy and not game.robot.grip_closed
        assert stages.index('finish_opening')<stages.index('finish_switching')<stages.index('returning')
        assert not any(p in stages for p in ('grasp_verify','grasp_place'))
        assert game.robot.current==game.front and game.robot.modes.mode=='impedance'


def test_finish_from_paused_returns_home_like_ghost_return_home():
    with graph([Game,Senses],PARAMS+['mock_step_sec:=0.15']) as (nodes,exe):
        game,senses,probe=nodes
        assert game.command('2')[0]
        until(exe,lambda:game.phase=='grasp_approach',timeout=12)
        assert game.command('stop')[0]
        until(exe,lambda:game.state=='paused' and not game.robot.busy and not game.ball_sequence.busy)
        game.robot.fault='stale fault from the paused round'
        assert game.command('finish')[0]
        until(exe,lambda:game.state=='idle',timeout=12)
        assert game.robot.current==game.front and game.mode==''


def test_target_loss_stops_sequence_without_closing():
    with graph([Game,Senses],PARAMS+['mock_step_sec:=0.15']) as (nodes,exe):
        game,senses,probe=nodes
        assert game.command('2')[0]
        until(exe,lambda:game.phase=='grasp_approach',timeout=12)
        senses.timer.cancel()
        until(exe,lambda:game.state=='paused')
        assert '定位丢失' in game.message
        assert not game.robot.grip_closed
        assert not game.ball_sequence.running


def test_stop_during_sequence_does_not_continue_to_close():
    with graph([Game,Senses],PARAMS+['mock_step_sec:=0.15']) as (nodes,exe):
        game,senses,probe=nodes
        assert game.command('2')[0]
        until(exe,lambda:game.phase=='grasp_approach',timeout=12)
        assert game.command('stop')[0]
        end=time.monotonic()+.8
        while time.monotonic()<end:exe.spin_once(timeout_sec=.01)
        assert game.state=='paused' and not game.robot.grip_closed
        assert not game.ball_sequence.running


@pytest.mark.parametrize('fraction',[.76,float('nan')])
def test_incomplete_hardware_plan_never_sends_or_closes(fraction):
    from types import SimpleNamespace
    from concurrent.futures import Future
    with graph([Game],PARAMS) as (nodes,exe):
        game,probe=nodes;s=game.ball_sequence
        game.mode='ball';s.running=True;s.phase='full_plan';s.deadline=time.monotonic()+5
        s.target=dict(center=[.3,0.,.1]);s.frozen=[.3,0.,.1];s.target_at=time.monotonic()
        s.feedback=([0.,.6,.8,0.,0.,0.],0.);s.feedback_at=time.monotonic()
        game.robot.mock=False;game.robot.fresh=lambda:True
        s.future=Future();s.future.set_result(SimpleNamespace(error_code=SimpleNamespace(val=1),fraction=fraction))
        game.robot.grip=lambda *args:pytest.fail('must not grip on partial plan')
        s.action.send_goal_async=lambda *args:pytest.fail('must not execute partial plan')
        s.tick()
        assert game.state=='paused' and '路径不完整' in game.message
        assert not s.running


def test_close_failure_never_lifts():
    with graph([Game,Senses],PARAMS) as (nodes,exe):
        game,senses,probe=nodes;grip=game.robot.grip
        def fail_close(closed,cb):
            if closed:return False,'test close failure'
            return grip(closed,cb)
        game.robot.grip=fail_close
        assert game.command('2')[0]
        until(exe,lambda:game.state=='paused',timeout=12)
        assert 'test close failure' in game.message
        assert game.ball_sequence.index==4


def test_late_action_acceptance_after_stop_is_cancelled_and_cannot_advance():
    from concurrent.futures import Future
    from types import SimpleNamespace
    with graph([Game],PARAMS) as (nodes,exe):
        game,probe=nodes;s=game.ball_sequence;cancelled=[];terminal=Future()
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:terminal,
                               cancel_goal_async=lambda:cancelled.append(True))
        accepted=Future();accepted.set_result(handle)
        s.running=True;s.submitting=True;game.robot.sequence_motion_busy=True
        game.command('stop')
        assert s.busy and not game.command('ready')[0]
        s.accepted(accepted)
        assert cancelled and s.busy
        terminal.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0)))
        assert not s.busy and not game.robot.sequence_motion_busy
        assert game.state=='paused' and s.index==0
