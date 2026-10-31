from armie.game import Game
from test_ros import graph,until


def test_ball_opens_before_tracking_and_never_accepts_close():
    with graph([Game],['mock_step_sec:=0.02','gripper_open_only:=true','ball_follow_only:=true']) as (nodes,exe):
        game=nodes[0];commands=[];original=game.robot.grip
        def grip(closed,callback):
            commands.append((closed,game.robot.servo))
            return original(closed,callback)
        game.robot.grip=grip
        assert game.command('2')[0]
        until(exe,lambda:game.robot.servo)
        assert commands==[(False,False)]
        assert not original(True,lambda *_:None)[0]
        game.command('stop')


def test_failed_open_pauses_before_table_motion():
    with graph([Game],['mock_step_sec:=0.02','gripper_open_only:=true','ball_follow_only:=true']) as (nodes,exe):
        game=nodes[0];initial=list(game.robot.current)
        def failed(closed,callback):
            assert not closed
            callback(False,'opening failed')
            return True,'accepted'
        game.robot.grip=failed
        assert game.command('2')[0]
        until(exe,lambda:game.state=='paused')
        assert game.message=='opening failed'
        assert game.robot.current==initial and not game.robot.servo
