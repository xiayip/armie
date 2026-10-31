from armie.pet import Pet
from test_actions import FakeRobot,executor
from test_ros import graph,until
from test_game import advance


class OffsetRobot(FakeRobot):
    def __init__(self):
        self.offset=[0.,0.,.035,.035,0.,0.]
        super().__init__()


def test_tolerance_is_joint_specific_and_stale_feedback_still_blocks():
    args=['motion_backend:=trajectory','motion_enabled:=true',
          'settled_position_tolerance:=[0.03,0.03,0.04,0.04,0.03,0.03]']
    with graph([Pet,OffsetRobot],args,executor) as (nodes,exe):
        pet,robot,_=nodes
        original=robot.state_pub.publish
        def shifted(msg):
            msg.reference.positions=[v+d for v,d in zip(msg.feedback.positions,robot.offset)]
            original(msg)
        robot.state_pub.publish=shifted
        until(exe,lambda:pet.motion.ready()=='')
        assert pet.motion.start('nod')[0]
        until(exe,lambda:not pet.motion.busy)
        assert len(robot.requests)==1
        for axis,value in [(0,.035),(2,.041),(3,.041)]:
            robot.offset=[0.]*6;robot.offset[axis]=value
            advance(exe,.6)
            assert not pet.motion.controller_stationary
            assert pet.motion.ready()!=''
            assert not pet.motion.start('nod')[0]
        robot.offset=[0.,0.,.035,.035,0.,0.]
        until(exe,lambda:pet.motion.ready()=='')
        robot.emit_telemetry=False
        advance(exe,.6)
        assert pet.motion.ready()!=''
        assert not pet.motion.start('nod')[0]
        assert len(robot.requests)==1
