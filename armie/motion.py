"""Reuse the existing FollowJointTrajectory / ExecuteTree action contracts.

Never switches controllers, writes MIT commands or changes gain/guard parameters.
Cancellation addresses only the goal owned by this client.
"""
import json
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from btcpp_ros2_interfaces.action import ExecuteTree
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from ghost_game_orchestrator.mock_trajectory import minimum_quintic_duration
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .common import finite, param
from .settling import SettlingWindow

JOINTS = [f'joint{i}' for i in range(1, 7)]
LOWER = [-2.8, 0., 0., -1.57, -1.57, -3.14]
UPPER = [2.8, 3.14, 3.14, 1.57, 1.57, 3.14]


def gesture_points(pose, gesture):
    if len(pose) != 6 or any(not math.isfinite(q) or not lo <= q <= hi
                             for q, lo, hi in zip(pose, LOWER, UPPER)):
        raise ValueError('invalid measured pose')
    if gesture not in {'greet', 'nod', 'tilt'}:
        raise ValueError('unknown gesture')
    # Small relative wrist movement; return to measured anchor, no home pose.
    index, offset = (5, .12) if gesture == 'tilt' else (3, .10)
    target = list(pose)
    target[index] += offset if pose[index] + offset <= UPPER[index] - .05 else -offset
    if not LOWER[index] + .05 <= target[index] <= UPPER[index] - .05:
        raise ValueError('gesture too close to joint limit')
    duration = minimum_quintic_duration(pose, target, [.12] * 6, 2.)
    # Same endpoint zero velocity/acceleration convention as ghost_game.
    return [(.2, list(pose)), (.2 + duration, target), (.2 + 2 * duration, list(pose))]


class Motion:
    def __init__(self, node, report):
        self.node, self.report = node, report
        self.prefix = param(node, 'controller_prefix', 'rebot_arm_rs')
        if self.prefix not in ('rebot_arm_rs', 'zephyr_arm'):
            raise ValueError('unsupported controller_prefix')
        self.backend = param(node, 'motion_backend', 'mock')
        self.enabled = param(node, 'motion_enabled', False)
        if self.backend not in {'mock', 'trajectory', 'bt'}:
            raise ValueError('motion_backend must be mock, trajectory or bt')
        self.action_name = param(node, 'trajectory_action',
                                 f'/{self.prefix}_impedance_trajectory_controller/follow_joint_trajectory')
        self.bt_name = param(node, 'bt_action', '/behavior_server')
        self.trees = json.loads(param(node, 'bt_trees_json', '{}'))
        if (not isinstance(self.trees, dict) or
                any(k not in {'greet', 'nod', 'tilt'} or not isinstance(v, str) or not v
                    for k, v in self.trees.items())):
            raise ValueError('bt_trees_json must map known gestures to commissioned tree IDs')
        self.limit_scale = finite(param(node, 'motion_limit_scale', 1.), 1., 1.5, 'motion limit scale')
        tolerance=param(node,'settled_position_tolerance',[.03]*6)
        if len(tolerance)!=6:raise ValueError('settled_position_tolerance requires six joint tolerances')
        self.settled_position_tolerance=[finite(v,.01,.05,'settled position tolerance') for v in tolerance]
        self.timeout = finite(param(node, 'motion_timeout_sec', 12.), 1., 60., 'motion timeout')
        self.guard = param(node, 'required_guard_node', f'/{self.prefix}_impedance_trajectory_guard')
        self.required = param(node, 'required_controllers', [
            f'{self.prefix}_mit_controller', f'{self.prefix}_impedance_controller',
            f'{self.prefix}_impedance_trajectory_controller'])
        self.manager = node.create_client(ListControllers, param(
            node, 'controller_manager', '/controller_manager') + '/list_controllers')
        self.traj = ActionClient(node, FollowJointTrajectory, self.action_name)
        self.bt = ActionClient(node, ExecuteTree, self.bt_name)
        self.pose, self.pose_at, self.stationary = None, 0., False
        self.joint_settling = SettlingWindow()
        self.reference_settling = SettlingWindow()
        self.feedback_settling = SettlingWindow()
        self.status_at, self.other_busy = 0., False
        self.matched_since, self.quiet_since = None, None
        self.controller_at, self.controller_stationary = 0., False
        self.reference_pose = None
        self.joint_sub = node.create_subscription(
            JointState, param(node, 'joint_states_topic', '/joint_states'),
            self.joints, qos_profile_sensor_data)
        self.status_sub = node.create_subscription(
            GoalStatusArray, self.action_name + '/_action/status',
            self.status, qos_profile_action_status_default)
        self.controller_sub = node.create_subscription(
            JointTrajectoryControllerState,
            self.action_name.rsplit('/', 1)[0] + '/controller_state',
            self.controller_state, qos_profile_sensor_data)
        self.busy, self.phase, self.handle = False, 'idle', None
        self.cancel_requested, self.cancel_sent = False, False
        self.deadline, self.gesture = 0., ''
        self.path_factory = None
        self.dance_path = False

    def joints(self, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp > 0 and not -.2 <= age <= .5:
                raise ValueError('stale joint timestamp')
            indices = [msg.name.index(j) for j in JOINTS]
            pose = [msg.position[i] for i in indices]
            velocity = [msg.velocity[i] for i in indices]
            if not all(math.isfinite(x) for x in pose + velocity):
                raise ValueError('nonfinite joints')
            self.pose, self.pose_at = pose, time.monotonic()
            self.stationary = self.joint_settling.update(self.pose_at, pose, velocity, stamp)
        except (ValueError, IndexError):
            self.pose_at, self.stationary = 0., False
            self.joint_settling.reset()

    def controller_state(self, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp > 0 and not -.2 <= age <= .5:
                raise ValueError('stale controller timestamp')
            indices = [msg.joint_names.index(j) for j in JOINTS]
            reference = [msg.reference.positions[i] for i in indices]
            feedback = [msg.feedback.positions[i] for i in indices]
            velocities = [msg.reference.velocities[i] for i in indices]
            velocities += [msg.feedback.velocities[i] for i in indices]
            now = time.monotonic()
            reference_quiet = self.reference_settling.update(now, reference, velocities[:6], stamp)
            feedback_quiet = self.feedback_settling.update(now, feedback, velocities[6:], stamp)
            valid = (reference_quiet and feedback_quiet
                     and all(abs(v) < .03 for v in velocities[:6])
                     and all(abs(a-b) <= limit for a,b,limit in zip(reference,feedback,self.settled_position_tolerance)))
            self.controller_at, self.controller_stationary = now, valid
            self.reference_pose = list(reference) if valid else None
        except (ValueError, IndexError):
            self.controller_stationary = False
            self.reference_pose = None
            self.reference_settling.reset()
            self.feedback_settling.reset()

    def status(self, msg):
        self.status_at = time.monotonic()
        self.other_busy = any(s.status in (0, 1, 2, 3) for s in msg.status_list)

    def ready(self):
        now = time.monotonic()
        if self.pose is None or now - self.pose_at > .5 or not self.stationary:
            return 'fresh stationary joint feedback required'
        # Action status is event-driven, not a periodic heartbeat. Mirror the
        # existing HMI bridge's discovery grace plus fresh JTC telemetry check.
        if self.matched_since is None or now - self.matched_since < 1. or self.other_busy:
            return 'trajectory status unmatched or another goal is active'
        if self.quiet_since is None or now - self.quiet_since < .5:
            return 'fresh settled JTC reference and feedback required'
        if self.node.count_publishers(self.action_name.rsplit('/', 1)[0] + '/joint_trajectory'):
            return 'direct trajectory publisher exists'
        nodes = {namespace.rstrip('/') + '/' + name
                 for name, namespace in self.node.get_node_names_and_namespaces()}
        if not self.guard or self.guard not in nodes:
            return 'existing impedance trajectory guard must be running'
        return ''

    def start(self, gesture):
        if self.busy:
            return False, 'motion busy or awaiting cancellation'
        if self.backend != 'mock' and not self.enabled:
            return False, 'motion_enabled is false'
        if self.backend == 'bt' and gesture not in self.trees:
            return False, 'no commissioned BT mapping for gesture'
        if self.backend != 'mock':
            reason = self.ready()
            if reason or not self.manager.service_is_ready():
                return False, reason or 'controller manager unavailable'
        self.busy, self.gesture, self.phase = True, gesture, 'preflight'
        self.cancel_requested, self.cancel_sent = False, False
        self.deadline = time.monotonic() + (1. if self.backend == 'mock' else 2.)
        if self.backend == 'mock':
            self.phase = 'mock'
        else:
            self.manager.call_async(ListControllers.Request()).add_done_callback(self.preflight_done)
        return True, 'accepted'

    def start_path(self, factory, *, dance=False):
        if self.busy:
            return False, 'motion busy'
        if self.backend not in {'mock', 'trajectory'}:
            return False, 'game paths require mock or trajectory backend'
        self.path_factory = factory
        self.dance_path = dance
        accepted, reason = self.start('nod')
        if not accepted:
            self.path_factory = None
            self.dance_path = False
        return accepted, reason

    def finish(self, success, message):
        self.busy, self.phase, self.handle = False, 'idle', None
        self.path_factory = None
        self.dance_path = False
        self.report(success, message)

    def preflight_done(self, future):
        if not self.busy:
            return
        if self.cancel_requested:
            self.finish(False, 'cancelled before submission')
            return
        try:
            active = {c.name for c in future.result().controller if c.state == 'active'}
            if not self.required or not set(self.required).issubset(active):
                raise ValueError('required impedance controller chain is not active')
            reason = self.ready()
            if reason:
                raise ValueError(reason)
            if self.backend == 'trajectory':
                client, goal = self.traj, FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = JOINTS
                # The existing impedance JTC interpolates from desired state.
                # Re-anchoring to measured position would unload its holding
                # error and can briefly reverse friction feed-forward. Keep
                # measured-position validation and the configured bounded
                # reference/feedback gate before continuing the settled reference.
                if self.reference_pose is None or any(
                        not lo <= q <= hi for q, lo, hi in zip(self.pose, LOWER, UPPER)):
                    raise ValueError('valid measured pose and settled reference required')
                anchor = list(self.reference_pose)
                points = self.path_factory(anchor) if self.path_factory else gesture_points(anchor, self.gesture)
                # Faster choreography has a separate envelope; ordinary moves retain
                # their original limits. J2/J3 stay below the existing 0.6 rad/s
                # controller position-rate limit. Controller parameters are untouched.
                velocity_limits = [.65, .6, .6, .65, .65, .65] if self.dance_path else [.4]*6
                acceleration_limit = 1.7 if self.dance_path else .65
                velocity_limits = [v*self.limit_scale for v in velocity_limits]
                acceleration_limit *= self.limit_scale
                previous = 0.
                for sample in points:
                    if len(sample) == 2:
                        at, pose = sample
                        velocity, acceleration = [0.]*6, [0.]*6
                    else:
                        at, pose, velocity, acceleration = sample
                    if (not math.isfinite(at) or not previous < at <= 60.
                            or any(len(v) != 6 for v in (pose, velocity, acceleration))
                            or not all(math.isfinite(v) for row in (pose, velocity, acceleration) for v in row)
                            or any(not lo <= q <= hi for q,lo,hi in zip(pose, LOWER, UPPER))
                            or any(abs(v) > limit for v, limit in zip(velocity, velocity_limits))
                            or any(abs(a) > acceleration_limit for a in acceleration)):
                        raise ValueError('invalid or excessive trajectory sample')
                    previous = at
                    point = JointTrajectoryPoint(positions=pose, velocities=velocity,
                                                 accelerations=acceleration)
                    ns = round(at*1e9)
                    point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
                    goal.trajectory.points.append(point)
            else:
                client, goal = self.bt, ExecuteTree.Goal()
                goal.target_tree = self.trees[self.gesture]
                goal.payload = ''  # Existing executor does not consume the payload.
            if not client.server_is_ready():
                raise ValueError('action server unavailable')
            duration = points[-1][0] + 6. if self.backend == 'trajectory' else self.timeout
            self.phase, self.deadline = 'sending', time.monotonic() + max(self.timeout, duration)
            client.send_goal_async(goal).add_done_callback(self.goal_done)
        except Exception as exc:
            if self.phase == 'sending':
                self.phase = 'uncertain'
                self.report(False, 'submission outcome unknown; verify action server before restart')
            else:
                self.finish(False, str(exc))

    def destroy(self):
        self.traj.destroy()
        self.bt.destroy()

    def goal_done(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.finish(False, 'goal rejected')
                return
            self.handle, self.phase = handle, 'executing'
            handle.get_result_async().add_done_callback(self.result_done)
            if self.cancel_requested:
                self.stop()
        except Exception:
            # Submission may have reached the server. Keep the gate latched.
            self.phase = 'uncertain'
            self.report(False, 'goal acceptance unknown; verify action server before restart')

    def result_done(self, future):
        try:
            result = future.result()
            ok = result.status == GoalStatus.STATUS_SUCCEEDED
            if self.backend == 'trajectory':
                ok = ok and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            else:
                ok = ok and result.result.node_status.status == 2  # NodeStatus.SUCCESS
            if self.cancel_requested:
                message = 'cancelled'
            elif ok:
                message = 'completed'
            elif self.backend == 'trajectory':
                message = (f'trajectory failed (status={result.status}, '
                           f'error={result.result.error_code}): {result.result.error_string}')
            else:
                message = f'BT failed (status={result.status}, node_status={result.result.node_status.status})'
            self.finish(ok and not self.cancel_requested, message)
        except Exception:
            self.phase = 'uncertain'
            self.report(False, 'result unknown; verify action server before restart')

    def stop(self):
        if not self.busy:
            return
        self.cancel_requested = True
        if self.phase == 'mock':
            self.finish(False, 'mock cancelled')
        elif self.handle is not None and not self.cancel_sent:
            self.cancel_sent = True
            self.phase = 'cancelling'
            try:
                self.handle.cancel_goal_async().add_done_callback(self.cancel_done)
            except Exception:
                self.report(False, 'cancel request failed; motion remains blocked')

    def cancel_done(self, future):
        try:
            if not future.result().goals_canceling:
                self.report(False, 'cancel not acknowledged; waiting for terminal action result')
        except Exception:
            self.report(False, 'cancel response unavailable; waiting for terminal action result')

    def tick(self):
        now = time.monotonic()
        matched = self.node.count_publishers(self.action_name + '/_action/status') > 0
        if not matched:
            self.matched_since = None
        elif self.matched_since is None:
            self.matched_since = now
        if (now - self.controller_at > .5 or not self.controller_stationary or
                self.other_busy or now - self.pose_at > .5 or not self.stationary):
            self.quiet_since = None
        elif self.quiet_since is None:
            self.quiet_since = now
        if not self.busy:
            return
        if self.phase == 'mock' and now >= self.deadline:
            self.finish(True, 'mock completed')
        elif self.backend != 'mock' and not self.cancel_requested:
            if now >= self.deadline or now - self.pose_at > .5 or now - self.controller_at > .5:
                self.stop()
                self.report(False, 'timeout or feedback loss; cancellation requested')
