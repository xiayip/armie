"""ROS service/action integration against fake controllers; never a real robot."""
import time
import pytest
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from armie.controller_modes import BASE,CHAINS,IMP,JTC,observed,switch_plan
from armie.game import Game
from armie.motion import JOINTS
from test_actions import FakeRobot,executor
from test_ros import graph,until


class FakeGameRobot(FakeRobot):
    def __init__(self):
        self.controllers={BASE:'active'}
        for names in CHAINS.values():
            for name in names:self.controllers[name]='inactive'
        self.controllers[IMP]=self.controllers[JTC]='active'
        self.position=[0.,.6,.8,.1,.1,0.]
        self.switch_requests=[]
        self.switch_delay=0.
        self.direct_commands=[]
        super().__init__()
        self.switch_service=self.create_service(SwitchController,'/controller_manager/switch_controller',self.switch_mode,callback_group=self.group)
        self.direct_sub=self.create_subscription(JointTrajectoryPoint,'/'+IMP+'/commands',self.direct,1)

    def list(self,request,response):
        response.controller=[ControllerState(name=n,state=s) for n,s in self.controllers.items()]
        return response

    def switch_mode(self,request,response):
        self.switch_requests.append(request)
        time.sleep(self.switch_delay)
        for n in request.deactivate_controllers:self.controllers[n]='inactive'
        for n in request.activate_controllers:self.controllers[n]='active'
        response.ok=True
        return response

    def direct(self,msg):
        self.direct_commands.append(list(msg.positions))
        if self.controllers[IMP]=='active' and self.controllers[JTC]=='inactive':self.position=list(msg.positions)

    def telemetry(self):
        if not self.emit_telemetry:return
        msg=JointState(name=JOINTS,position=self.position,velocity=[0.]*6)
        msg.header.stamp=self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)
        if self.controllers[JTC]=='active':
            state=JointTrajectoryControllerState(joint_names=JOINTS)
            state.header.stamp=msg.header.stamp
            state.reference.positions=state.feedback.positions=self.position
            state.reference.velocities=state.feedback.velocities=[0.]*6
            self.state_pub.publish(state)

    def execute(self,handle):
        result=super().execute(handle)
        if not handle.is_cancel_requested:self.position=list(handle.request.trajectory.points[-1].positions)
        return result


def ready(game,exe):
    until(exe,lambda:game.robot.motion.ready()=='' and game.robot.modes.quiet_since is not None)
    assert game.command('ready')[0]
    until(exe,lambda:game.state=='idle')


PARAMS=['motion_backend:=trajectory','motion_enabled:=true',
        'front_pose_json:=\'[0.0,0.6,0.8,0.1,0.1,0.0]\'','servo_commissioned:=true']


def test_ready_requires_post_action_reference_and_feedback_quiet_window():
    with graph([Game,FakeGameRobot],PARAMS,executor) as (nodes,exe):
        game,robot,probe=nodes
        ready(game,exe)
        assert game.robot.motion.ready()==''
        assert game.robot.stationary()
        assert not game.robot.busy


@pytest.mark.parametrize('speed',[1.,1.6])
def test_dance_action_preserves_nonzero_velocities_and_returns_home(speed):
    with graph([Game,FakeGameRobot],PARAMS+[f'dance_speed_scale:={speed}'],executor) as (nodes,exe):
        game,robot,probe=nodes
        ready(game,exe)
        assert game.command('3')[0]
        until(exe,lambda:game.state=='idle',timeout=8.)
        wave=max(robot.requests,key=lambda g:len(g.trajectory.points)).trajectory.points
        assert len(wave)>100
        assert any(abs(p.velocities[1])>.05 and abs(p.velocities[2])>.05 for p in wave)
        assert max(p.positions[1] for p in wave)-game.front[1]>.35
        assert max(p.positions[2] for p in wave)-game.front[2]>.45
        assert list(wave[-1].positions)==game.front
        assert list(wave[-1].velocities)==[0.]*6


def test_soft_mode_verified_switch_and_release_restores_current_pose():
    with graph([Game,FakeGameRobot],PARAMS,executor) as (nodes,exe):
        game,robot,probe=nodes
        ready(game,exe)
        assert game.command('1')[0]
        until(exe,lambda:game.phase=='dragging')
        assert observed(robot.list(None,type('R',(),{})()).controller)=='damping'
        request=robot.switch_requests[-1]
        assert list(request.activate_controllers)==['rebot_arm_rs_damping_controller']
        assert BASE not in request.deactivate_controllers
        robot.position=[0.,.7,.9,.1,.1,0.]
        game.command('finish')
        until(exe,lambda:game.phase=='waiting_release')
        before=len(robot.switch_requests)
        for _ in range(20):exe.spin_once(timeout_sec=.01)
        assert len(robot.switch_requests)==before
        game.command('release')
        until(exe,lambda:game.state=='idle',timeout=6.)
        assert robot.position==game.front
        # Starting point of the return trajectory is the dragged measured pose.
        assert list(robot.requests[-1].trajectory.points[0].positions)==[0.,.7,.9,.1,.1,0.]


def test_mode_service_timeout_stays_latched_even_if_late_switch_completes():
    with graph([Game,FakeGameRobot],PARAMS,executor) as (nodes,exe):
        game,robot,probe=nodes
        ready(game,exe)
        robot.switch_delay=.5
        assert game.command('1')[0]
        until(exe,lambda:game.robot.modes.phase=='switch')
        game.robot.modes.deadline=time.monotonic()-.1
        until(exe,lambda:game.state=='paused')
        until(exe,lambda:robot.controllers['rebot_arm_rs_damping_controller']=='active')
        assert game.robot.modes.phase=='quarantine' and game.robot.busy
        assert not game.command('ready')[0]


def test_servo_entry_and_exit_only_change_the_upstream_jtc():
    with graph([Game,FakeGameRobot],PARAMS,executor) as (nodes,exe):
        game,robot,probe=nodes
        ready(game,exe)
        assert game.command('4')[0]
        until(exe,lambda:game.robot.servo)
        request=robot.switch_requests[-1]
        assert list(request.deactivate_controllers)==[JTC]
        assert not request.activate_controllers
        assert robot.direct_commands
        game.command('finish')
        until(exe,lambda:game.state=='idle',timeout=6.)
        assert robot.controllers[JTC]=='active'
        assert all(BASE not in r.deactivate_controllers for r in robot.switch_requests)


def test_unknown_controller_owner_is_rejected():
    controllers=[ControllerState(name=BASE,state='active'),
                 ControllerState(name=IMP,state='active'),ControllerState(name=JTC,state='active'),
                 ControllerState(name='foreign',state='active',claimed_interfaces=['joint1/position'])]
    import pytest
    with pytest.raises(ValueError):switch_plan(controllers,'damping')


def test_official_gripper_action_payload_and_foreign_goal_guard():
    from rclpy.action import ActionServer
    from control_msgs.action import ParallelGripperCommand
    with graph([Game,FakeGameRobot],PARAMS+['gripper_enabled:=true'],executor) as (nodes,exe):
        game,robot,probe=nodes
        requests=[]
        def execute(handle):
            requests.append(handle.request)
            result=ParallelGripperCommand.Result()
            result.state=handle.request.command
            result.reached_goal=True
            handle.succeed()
            return result
        server=ActionServer(robot,ParallelGripperCommand,
                            '/rebot_arm_rs_gripper_controller/gripper_cmd',execute,callback_group=robot.group)
        try:
            ready(game,exe)
            until(exe,lambda:game.robot.gripper_matched_since is not None and time.monotonic()-game.robot.gripper_matched_since>=1.)
            outcomes=[]
            assert game.robot.grip(False,lambda *r:outcomes.append(r))[0]
            until(exe,lambda:bool(outcomes))
            assert outcomes[0][0]
            assert list(requests[0].command.name)==['gripper_command_joint']
            assert list(requests[0].command.position)==[0.]
            assert list(requests[0].command.velocity)==[.15]
            game.robot.gripper_other_busy=True
            assert not game.robot.grip(True,lambda *r:None)[0]
        finally:server.destroy()


def test_zephyr_controller_mapping_preserves_mode_and_unknown_owner_guards():
    from armie.controller_modes import controller_names
    names=controller_names('zephyr_arm')
    controllers=[ControllerState(name=names[BASE],state='active')]
    controllers += [ControllerState(name=names[n],state='active' if n in (IMP,JTC) else 'inactive')
                    for n in names if n != BASE]
    assert observed(controllers,'zephyr_arm')=='impedance'
    activate,deactivate=switch_plan(controllers,'damping','zephyr_arm')
    assert activate==['zephyr_arm_damping_controller']
    assert deactivate==['zephyr_arm_impedance_trajectory_controller','zephyr_arm_impedance_controller']
    assert names[BASE] not in deactivate
    controllers.append(ControllerState(name='foreign_controller',state='active',
                                      claimed_interfaces=['zephyr_arm_mit_controller/joint1/position']))
    with pytest.raises(ValueError,match='unknown arm owner'):
        observed(controllers,'zephyr_arm')
    controllers.pop()
    controllers.append(ControllerState(name=BASE,state='active'))
    with pytest.raises(ValueError,match='mixed'):
        observed(controllers,'zephyr_arm')


def test_zephyr_game_routes_to_matching_controller_names():
    with graph([Game],['controller_prefix:=zephyr_arm']) as (nodes,exe):
        game=nodes[0]
        assert game.robot.command_pub.topic_name=='/zephyr_arm_impedance_controller/commands'
        assert set(game.robot.modes.status)=={'zephyr_arm_impedance_trajectory_controller',
                                             'zephyr_arm_trajectory_controller'}
        assert game.robot.modes.imp=='zephyr_arm_impedance_controller'


def test_ready_table_uses_recorded_view_without_front_detour():
    table=[0.,.6,.8,.15,.1,0.]
    import json
    with graph([Game,FakeGameRobot],PARAMS+["table_pose_json:='"+json.dumps(table)+"'","motion_limit_scale:=1.5"],executor) as (nodes,exe):
        game,robot,probe=nodes
        until(exe,lambda:game.robot.motion.ready()=='' and game.robot.modes.quiet_since is not None)
        assert game.command('ready_table')[0]
        until(exe,lambda:game.state=='idle')
        assert all(list(goal.trajectory.points[-1].positions)==table for goal in robot.requests)
        assert game.robot.tracking_speed==pytest.approx(.3)
        assert game.robot.tracking_acceleration==pytest.approx(.975)
