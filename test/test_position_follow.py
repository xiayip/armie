import math
from types import SimpleNamespace
import pytest
from controller_manager_msgs.msg import ControllerState
from armie.controller_modes import BASE,KNOWN,IMP,JTC,POS,CHAINS,observed,switch_plan
from armie.game import Game
from test_ros import graph


def test_position_chain_roundtrip_retains_base_and_rejects_foreign_owner():
    controllers=[ControllerState(name=n,state='active' if n in (BASE,IMP) else 'inactive') for n in {BASE,*KNOWN}]
    a,d=switch_plan(controllers,'position')
    assert a==['rebot_arm_rs_position_adapter',POS] and d==[IMP]
    for c in controllers:
        if c.name in a:c.state='active'
        if c.name in d:c.state='inactive'
    assert observed(controllers)=='position'
    a,d=switch_plan(controllers,'impedance')
    assert a==[IMP,JTC] and d==[POS,'rebot_arm_rs_position_adapter']
    assert BASE not in d
    controllers.append(ControllerState(name='foreign',state='active',claimed_interfaces=['joint1/position']))
    with pytest.raises(ValueError):switch_plan(controllers,'impedance')


def test_position_rolling_tail_stops_within_reserved_workspace_and_hold_is_zero():
    with graph([Game],['ball_controller:=position','ball_tracking_speed:=0.9','ball_tracking_acceleration:=1.5']) as (nodes,exe):
        g=nodes[0];g.timer.cancel();r=g.robot;sent=[]
        r.position_pub=SimpleNamespace(publish=sent.append)
        r.mock=False;r.motion.enabled=True;r.motion.pose=[0.,.6,.8,0.,0.,0.]
        r.fresh=lambda:True;r.modes.mode='position';r.servo=True
        r.reference=r.current;r.tracking_anchor=r.current;r.set_tracking_profile(True)
        for i in range(140):
            target=r.reference[:];target[3]=.25 if i<70 else -.25;target[4]=.35 if i<70 else -.35
            assert r.track(target,.025)
            msg=sent[-1]
            assert list(msg.points[-1].velocities)==[0.]*6
            for p in msg.points:
                assert abs(p.positions[3])<=.25+1e-8 and abs(p.positions[4])<=.35+1e-8
                assert max(abs(v) for v in p.velocities)<=.9+1e-9
            for p,q in zip(msg.points,msg.points[1:]):
                dt=q.time_from_start.sec+q.time_from_start.nanosec*1e-9-p.time_from_start.sec-p.time_from_start.nanosec*1e-9
                assert dt>0
                assert max(abs(a-b)/dt for a,b in zip(p.velocities,q.velocities))<=1.5+1e-5
        r.hold();assert not r.servo
        assert list(sent[-1].points[-1].positions)==r.current
        assert list(sent[-1].points[-1].velocities)==[0.]*6


def test_game_routes_only_ball_to_position():
    with graph([Game],['ball_controller:=position']) as (nodes,exe):
        g=nodes[0];g.timer.cancel();calls=[]
        g.robot.mode=lambda target,cb:(calls.append(target) or (True,'accepted'))
        g.mode='ball';g.begin_tracking();assert calls[-1]=='position'
        g.mode='face';g.begin_tracking();assert calls[-1]=='servo'
