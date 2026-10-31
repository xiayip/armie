import time
import pytest
from armie.game import Game
from armie.game_core import path_points
from ghost_game_orchestrator.face_tracking import make_face_sample
from test_ros import graph


def test_ball_gain_reaches_faster_command_and_face_profile_is_preserved():
    with graph([Game],['ball_tracking_gain:=2.0','ball_tracking_speed:=0.9','ball_tracking_acceleration:=1.5','motion_limit_scale:=1.5']) as (nodes,exe):
        g=nodes[0];g.timer.cancel();g.mode='ball';g.tracking_started()
        assert g.robot.tracking_speed==.9 and g.robot.tracking_acceleration==1.5
        g.robot.reference=g.robot.current;targets=[]
        g.robot.track=lambda target,dt:targets.append(target)
        now=time.monotonic();g.sample=make_face_sample(now,1.2,1.,.1,.1,2,2,.9)
        g.tracking_tick(now,.025)
        assert (targets[-1][4]-g.robot.reference[4])/.025==pytest.approx(-.4)
        g.mode='face';g.tracking_started()
        assert g.robot.tracking_speed==pytest.approx(.3)
        assert g.robot.tracking_acceleration==pytest.approx(.975)
        g.sample=make_face_sample(time.monotonic(),1.2,1.,.1,.1,2,2,.9)
        g.tracking_tick(time.monotonic(),.025)
        assert (targets[-1][4]-g.robot.reference[4])/.025==pytest.approx(-.1)


def test_faster_jtc_segments_respect_speed_and_acceleration():
    import math
    start=[0.,.6,.8,0.,0.,0.]
    for i in range(101):
        target=start[:];target[0]=10**(-5+5*i/100)
        points=path_points(start,[target],.48,.25,.975)
        dt=points[-1][0]-.2;distance=target[0]
        assert 1.875*distance/dt<=.48+1e-9
        assert 10/math.sqrt(3)*distance/dt**2<=.975+1e-9


def test_interpolated_speed_cannot_bypass_ordinary_motion_envelope():
    with graph([Game]) as (nodes,exe):
        g=nodes[0]
        accepted,reason=g.robot.move([g.front],lambda *_:None,speed=.48,minimum=.25)
        assert not accepted and 'envelope' in reason


def test_ball_sends_bounded_velocity_and_hold_clears_it():
    from types import SimpleNamespace
    with graph([Game],['ball_velocity_feedforward:=true','ball_tracking_speed:=0.9','ball_tracking_acceleration:=1.5']) as (nodes,exe):
        g=nodes[0];g.timer.cancel();r=g.robot
        sent=[];r.command_pub=SimpleNamespace(publish=sent.append)
        r.mock=False;r.motion.enabled=True;r.motion.pose=[0.,.6,.8,0.,0.,0.]
        r.fresh=lambda:True;r.servo=True;r.modes.mode='servo'
        r.reference=r.current;r.tracking_anchor=r.current;r.set_tracking_profile(True)
        target=r.current;target[4]=.3
        assert r.track(target,.025)
        assert 0<sent[-1].velocities[4]<=1.5*.025
        assert list(sent[-1].velocities)==r.tracking_velocity
        r.hold();assert list(sent[-1].velocities)==[0.]*6
        r.set_tracking_profile(False);assert not r.tracking_feedforward
