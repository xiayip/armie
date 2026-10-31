import math
import pytest
from armie.game_core import path_points
from armie.game import Game
from test_game_hardware_adapter import FakeGameRobot,PARAMS,ready
from test_actions import executor
from test_ros import graph,until

FRONT=[-.157,.2,.2,0.,0.,0.]
TABLE=[.35244259238243103,.6297186017036438,.5457301735877991,-.8513855934143066,-.02147648110985756,.0019174760673195124]


def duration(goal):
    end=goal.trajectory.points[-1].time_from_start
    return end.sec+end.nanosec*1e-9


def test_ready_and_ball_preparation_submit_twice_speed_trajectories():
    with graph([Game,FakeGameRobot],PARAMS+['pose_transition_speed_scale:=2.0','motion_limit_scale:=1.5'],executor) as (nodes,exe):
        game,robot,_=nodes
        game.front=FRONT;game.table=TABLE;robot.position=TABLE[:]
        ready(game,exe)
        expected=path_points(TABLE,[FRONT],.24,.5,.975)[-1][0]
        original=path_points(TABLE,[FRONT],.12,1.)[-1][0]
        assert duration(robot.requests[-1])==pytest.approx(expected,abs=1e-8)
        assert expected-.2==pytest.approx((original-.2)/2)
        assert game.command('ready_table')[0]
        until(exe,lambda:game.state=='idle')
        assert duration(robot.requests[-1])==pytest.approx(expected,abs=1e-8)
        ready(game,exe)
        assert game.command('2')[0]
        until(exe,lambda:game.robot.servo)
        assert duration(robot.requests[-1])==pytest.approx(expected,abs=1e-8)
        assert list(robot.requests[-1].trajectory.points[-1].positions)==TABLE
        game.command('stop')


def test_short_moves_keep_interpolated_acceleration_bounded_at_double_speed():
    start=[0.,.6,.8,0.,0.,0.]
    for i in range(101):
        distance=10**(-5+5*i/100)
        target=list(start);target[0]=distance
        points=path_points(start,[target],.24,.5,.975)
        t=points[-1][0]-points[0][0]
        assert 1.875*distance/t<=.24+1e-10
        assert (10/math.sqrt(3))*distance/t**2<=.975+1e-10
    target=start[:];target[0]=.064
    assert path_points(start,[target],.24,.5,.975)[-1][0]>.7


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf')])
def test_invalid_acceleration_rejected(value):
    with pytest.raises(ValueError):path_points(FRONT,[TABLE],.24,.5,value)
