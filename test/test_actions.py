"""Actual ROS action transport tests; fake server, isolated domain, no hardware."""
import time

import pytest
from btcpp_ros2_interfaces.action import ExecuteTree
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from armie.motion import JOINTS
from armie.pet import Pet
from test_ros import graph, until, call


class FakeRobot(Node):
    def __init__(self):
        super().__init__('rebot_arm_rs_impedance_trajectory_guard')
        self.group = ReentrantCallbackGroup()
        self.requests, self.bt_requests, self.cancelled = [], [], 0
        self.run_seconds, self.accept_delay = .2, 0.
        self.chain_active, self.emit_telemetry, self.reference_offset = True, True, 0.
        self.list_service = self.create_service(ListControllers, '/controller_manager/list_controllers', self.list)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(JointTrajectoryControllerState,
            '/rebot_arm_rs_impedance_trajectory_controller/controller_state', qos_profile_sensor_data)
        self.action = ActionServer(
            self, FollowJointTrajectory,
            '/rebot_arm_rs_impedance_trajectory_controller/follow_joint_trajectory',
            self.execute, goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=self.group)
        self.bt = ActionServer(self, ExecuteTree, '/behavior_server', self.execute_bt,
                               cancel_callback=self.cancel, callback_group=self.group)
        self.timer = self.create_timer(.03, self.telemetry, callback_group=self.group)

    def list(self, request, response):
        response.controller = [ControllerState(name=name, state='active' if self.chain_active else 'inactive')
                               for name in ['rebot_arm_rs_mit_controller',
                                            'rebot_arm_rs_impedance_controller',
                                            'rebot_arm_rs_impedance_trajectory_controller']]
        return response

    def telemetry(self):
        if not self.emit_telemetry:
            return
        pose = [0., .6, .8, .1, .1, 0.]
        msg = JointState(name=JOINTS, position=pose, velocity=[0.] * 6)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)
        state = JointTrajectoryControllerState(joint_names=JOINTS)
        state.header.stamp = msg.header.stamp
        state.reference.positions = [x + self.reference_offset for x in pose]
        state.feedback.positions = pose
        state.reference.velocities = state.feedback.velocities = [0.] * 6
        self.state_pub.publish(state)

    def goal(self, request):
        from rclpy.action import GoalResponse
        time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancelled += 1
        return CancelResponse.ACCEPT

    def execute(self, handle):
        self.requests.append(handle.request)
        deadline = time.monotonic() + self.run_seconds
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                return FollowJointTrajectory.Result(error_code=0)
            time.sleep(.01)
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0)

    def execute_bt(self, handle):
        self.bt_requests.append(handle.request)
        handle.succeed()
        result = ExecuteTree.Result()
        result.node_status.status = 2
        return result

    def destroy_node(self):
        self.action.destroy()
        self.bt.destroy()
        return super().destroy_node()


def executor():
    return MultiThreadedExecutor(num_threads=4)


def ready(pet, exe):
    until(exe, lambda: pet.motion.ready() == '', timeout=4.)


def test_trajectory_action_preflight_completion_and_late_cancel():
    with graph([Pet, FakeRobot], ['motion_backend:=trajectory', 'motion_enabled:=true'], executor) as (nodes, exe):
        pet, robot, probe = nodes
        ready(pet, exe)
        assert pet.motion.start('nod')[0]
        until(exe, lambda: not pet.motion.busy)
        assert len(robot.requests) == 1
        goal = robot.requests[0]
        assert len(goal.trajectory.points) == 3
        assert list(goal.trajectory.joint_names) == JOINTS
        assert goal.trajectory.points[0].positions == goal.trajectory.points[-1].positions
        assert pet.expression == 'happy'

        ready(pet, exe)
        robot.accept_delay, robot.run_seconds = .3, 2.
        assert pet.motion.start('tilt')[0]
        until(exe, lambda: pet.motion.phase == 'sending')
        stop = probe.create_client(Trigger, '/armie/stop')
        assert call(exe, stop).success
        until(exe, lambda: not pet.motion.busy)
        assert robot.cancelled == 1 and pet.state == 'stopped'


def test_controller_inactive_and_stale_reference_prevent_goal_submission():
    with graph([Pet, FakeRobot], ['motion_backend:=trajectory', 'motion_enabled:=true'], executor) as (nodes, exe):
        pet, robot, probe = nodes
        ready(pet, exe)
        robot.chain_active = False
        assert pet.motion.start('nod')[0]
        until(exe, lambda: not pet.motion.busy)
        assert not robot.requests and pet.expression == 'confused'
        robot.chain_active, robot.reference_offset = True, .5
        until(exe, lambda: not pet.motion.controller_stationary and pet.motion.quiet_since is None)
        assert pet.motion.start('nod')[0] is False
        assert not robot.requests


def test_feedback_loss_cancels_owned_action():
    with graph([Pet, FakeRobot], ['motion_backend:=trajectory', 'motion_enabled:=true'], executor) as (nodes, exe):
        pet, robot, probe = nodes
        ready(pet, exe)
        robot.run_seconds = 2.
        assert pet.motion.start('nod')[0]
        until(exe, lambda: pet.motion.phase == 'executing')
        robot.emit_telemetry = False
        until(exe, lambda: not pet.motion.busy)
        assert robot.cancelled == 1 and pet.expression == 'confused'


def test_new_path_preserves_settled_reference_despite_measured_holding_error():
    with graph([Pet, FakeRobot], ['motion_backend:=trajectory', 'motion_enabled:=true'], executor) as (nodes, exe):
        pet, robot, probe = nodes
        robot.reference_offset = .02
        ready(pet, exe)
        measured = list(pet.motion.pose)
        reference = list(pet.motion.reference_pose)
        assert pet.motion.start('nod')[0]
        until(exe, lambda: not pet.motion.busy)
        points = robot.requests[-1].trajectory.points
        assert list(points[0].positions) == reference
        assert list(points[-1].positions) == reference
        assert list(points[0].positions) != measured
        robot.reference_offset = .04
        until(exe, lambda: not pet.motion.controller_stationary)
        assert not pet.motion.start('nod')[0]
        assert len(robot.requests) == 1


def test_trajectory_failure_exposes_controller_reason():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from armie.motion import Motion
    motion = Motion.__new__(Motion)
    motion.backend, motion.cancel_requested = 'trajectory', False
    reports = []
    motion.finish = lambda ok, message: reports.append((ok, message))
    result = Future()
    result.set_result(SimpleNamespace(status=6, result=FollowJointTrajectory.Result(
        error_code=-5, error_string='joint3 goal tolerance exceeded')))
    motion.result_done(result)
    assert reports == [(False, 'trajectory failed (status=6, error=-5): joint3 goal tolerance exceeded')]


def test_bt_uses_only_explicit_tree_mapping_and_checks_result():
    with graph([Pet, FakeRobot], ['motion_backend:=bt', 'motion_enabled:=true',
                                 'bt_trees_json:=\'{"nod":"ArmieCommissionedNod"}\''], executor) as (nodes, exe):
        pet, robot, probe = nodes
        ready(pet, exe)
        assert pet.motion.start('nod')[0]
        until(exe, lambda: not pet.motion.busy)
        assert robot.bt_requests[0].target_tree == 'ArmieCommissionedNod'
        assert robot.bt_requests[0].payload == ''
        assert pet.expression == 'happy' and not robot.requests
