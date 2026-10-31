"""RGB-D ball workflow. Plans never double as permission to execute a partial path."""
import math
import time
from collections import deque

import numpy as np
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .ball_grasp_geometry import locate_ball, preview_waypoints, wrapup_waypoints, grasp_evidence, WorldStillGate
from .common import finite, param
from .controller_modes import observed
from .motion import LOWER, UPPER
from controller_manager_msgs.srv import ListControllers

JOINTS = ['joint'+str(i) for i in range(1, 7)]


def timed_points(trajectory, current):
    """Keep every planned sample; only slow time, never replace with joint shortcuts."""
    if list(trajectory.joint_names) != JOINTS or len(trajectory.points) < 2:
        raise ValueError('invalid Cartesian trajectory')
    rows = trajectory.points
    if max(abs(a-b) for a,b in zip(rows[0].positions, current)) > .01:
        raise ValueError('arm moved since planning')
    for p in rows:
        if any(len(v) != 6 for v in (p.positions,p.velocities,p.accelerations)):
            raise ValueError('planner derivatives missing')
        if not all(math.isfinite(x) for row in (p.positions,p.velocities,p.accelerations) for x in row):
            raise ValueError('non-finite path')
        if not all(lo+.05 < q < hi-.05 for q,lo,hi in zip(p.positions,LOWER,UPPER)):
            raise ValueError('path enters joint limit margin')
    scale = max(2., max(abs(v) for p in rows for v in p.velocities)/.04*1.01,
                math.sqrt(max(abs(a) for p in rows for a in p.accelerations)/.12)*1.01)
    points=[]; previous=-1
    for p in rows:
        t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9
        ns=round((.3+scale*t)*1e9)
        if t<0 or ns<=previous or ns>60_000_000_000:raise ValueError('invalid trajectory duration')
        previous=ns
        point=JointTrajectoryPoint(positions=list(p.positions),velocities=[v/scale for v in p.velocities],
                                   accelerations=[a/scale**2 for a in p.accelerations])
        point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1_000_000_000)
        points.append(point)
    if max(abs(v) for v in points[-1].velocities)>.001:raise ValueError('path does not end stopped')
    return points


class BallSequence:
    def __init__(self, game):
        self.g=game;self.robot=game.robot
        self.enabled=param(game,'ball_full_sequence',False)
        self.nominal=param(game,'allow_nominal_tcp_trial',False)
        self.table_id=param(game,'grasp_table_object','armie_observed_table')
        self.gate=WorldStillGate(game.ball_still_sec)
        self.depth=deque(maxlen=12);self.info=None;self.joints=None
        self.target=None;self.target_at=0.;self.stable=False;self.last_stamp=0.
        self.running=False;self.phase='';self.future=None;self.handle=None;self.submitting=False
        self.cancelled=False;self.feedback_at=0.;self.feedback=None;self.index=0;self.waypoints=[]
        self.deadline=0.;self.settled_since=None;self.frozen=None;self.generation=0
        # Success needs both a finger stopped short of closed and a live camera
        # that no longer sees the ball at its table position.
        self.verify_sec=finite(param(game,'grasp_verify_sec',1.),.5,5.,'grasp verify duration')
        self.show_sec=finite(param(game,'grasp_show_sec',1.5),0.,5.,'grasp show duration')
        self.empty_finger=finite(param(game,'grasp_empty_finger_position',.92),.5,1.,'empty finger position')
        self.min_finger=finite(param(game,'grasp_min_finger_position',.1),0.,.5,'minimum finger position')
        self.mock_success=param(game,'mock_grasp_success',True)
        self.finger_state=None;self.home=None;self.success=False
        self.verify_since=0.;self.verify_frames=0;self.verify_on_table=False;self.show_until=0.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,game)
        self.plan=game.create_client(GetCartesianPath,'/compute_cartesian_path')
        self.scene=game.create_client(GetPlanningScene,'/get_planning_scene')
        self.action=ActionClient(game,FollowJointTrajectory,'/'+self.robot.modes.pos+'/follow_joint_trajectory')
        for key,topic,typ in [('depth','/camera/depth/image_raw',Image),('info','/camera/color/camera_info',CameraInfo),('joints','/joint_states',JointState)]:
            game.create_subscription(typ,topic,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
        game.create_subscription(JointTrajectoryControllerState,'/'+self.robot.modes.pos+'/controller_state',self.receive_feedback,qos_profile_sensor_data)

    @property
    def busy(self):return self.running or self.submitting or self.handle is not None

    def receive(self,key,msg):
        if key=='depth':self.depth.append(msg)
        else:setattr(self,key,msg)
        if key=='joints' and self.robot.gripper_joint in msg.name:
            i=msg.name.index(self.robot.gripper_joint)
            if i<len(msg.position) and math.isfinite(msg.position[i]):self.finger_state=(msg.position[i],time.monotonic())

    def finger(self):
        """Normalized finger position, 0 open .. 1 fully closed; None when stale."""
        if self.robot.mock:return (.55 if self.mock_success else 1.) if self.robot.grip_closed else 0.
        if self.finger_state is None or time.monotonic()-self.finger_state[1]>.3:return None
        return self.finger_state[0]

    def receive_feedback(self,msg):
        try:
            indices=[msg.joint_names.index(j) for j in JOINTS]
            q=[msg.feedback.positions[i] for i in indices]
            ref=[msg.reference.positions[i] for i in indices]
            if not all(math.isfinite(v) for v in q+ref):return
            if abs(self.g.get_clock().now().nanoseconds*1e-9-self.stamp(msg))>.25:return
            self.feedback=(q,max(abs(a-b) for a,b in zip(q,ref)))
            self.feedback_at=time.monotonic()
        except (ValueError,IndexError):return

    @staticmethod
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def observe(self,d):
        if not self.enabled or self.g.mode!='ball':return
        try:
            stamp=d['stamp'];now=self.g.get_clock().now().nanoseconds*1e-9
            if stamp<=self.last_stamp:return
            self.last_stamp=stamp
            fresh=d.get('valid') is True and 0<=now-stamp<=.15 and (self.robot.mock or d.get('source')=='camera')
            # Any fresh valid frame proves the camera was looking, ball or not.
            if self.phase=='verify' and fresh:self.verify_frames+=1
            if not fresh or not d.get('ball'):raise ValueError('target unavailable')
            if self.robot.mock:
                carried=self.frozen is not None and self.index>=5 and self.robot.grip_closed and self.mock_success
                target=dict(center=(np.asarray(self.frozen)+[0.,0.,.12]).tolist() if carried else [.3,0.,.1],radius=.025)
            else:
                if not self.depth or self.info is None:raise ValueError('RGB-D unavailable')
                m=min(self.depth,key=lambda x:abs(self.stamp(x)-stamp))
                if abs(self.stamp(m)-stamp)>.05:return
                if (m.width,m.height,m.header.frame_id)!=(self.info.width,self.info.height,self.info.header.frame_id):raise ValueError('depth not aligned')
                if m.encoding!='16UC1':raise ValueError('expected millimetre depth')
                depth=np.frombuffer(m.data,dtype='>u2' if m.is_bigendian else '<u2').reshape(m.height,m.step//2)[:,:m.width]*.001
                tr=self.buffer.lookup_transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                target=locate_ball(depth,self.info.k,d['ball'],[tr.translation.x,tr.translation.y,tr.translation.z],
                    [tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
            self.target=target;self.target_at=time.monotonic()
            if self.phase=='verify' and np.linalg.norm(np.asarray(target['center'])-self.frozen)<.02:self.verify_on_table=True
            self.stable=self.gate.update(target['center'],stamp)
        except Exception:
            self.stable=False;self.gate.reset()

    def target_valid(self):
        return self.target is not None and time.monotonic()-self.target_at<=.2 and (
            self.frozen is None or np.linalg.norm(np.array(self.target['center'])-self.frozen)<.01)

    def begin(self):
        if self.busy or not self.stable or not self.target_valid():return
        if not self.g.grasp_enabled or self.robot.gripper_open_only or (not self.robot.mock and (not self.nominal or not self.robot.gripper_enabled)):
            self.g.pause('完整抓取配置未启用：需明确名义 TCP 试验和夹爪配置');return
        self.robot.hold();self.running=True;self.cancelled=False;self.generation=self.g.generation
        self.frozen=list(self.target['center']);self.index=0;self.settled_since=None
        self.success=False;self.home=None
        self.phase='settle';self.deadline=time.monotonic()+5
        self.g.set_state('active','grasp_settling','小球静止，保持并检查完整抓取路径')

    def fail(self,reason):
        self.g.pause('抓球暂停：'+reason)

    def stop(self):
        self.cancelled=True;self.running=False;self.stable=False;self.gate.reset();self.frozen=None
        if self.future is not None:self.future.cancel();self.future=None
        if self.handle is not None:self.handle.cancel_goal_async()
        # A late accepted goal is cancelled by accepted(); busy stays latched until terminal.

    def request_plan(self,waypoints,phase):
        if self.joints is None or self.g.get_clock().now().nanoseconds*1e-9-self.stamp(self.joints)>.2:
            raise ValueError('joint feedback stale')
        if not self.plan.service_is_ready():raise ValueError('MoveIt unavailable')
        req=GetCartesianPath.Request();req.header.frame_id='base_link';req.start_state.joint_state=self.joints;req.start_state.is_diff=True
        req.group_name='zephyr_arm_tcp';req.link_name='omni_picker_tcp';req.max_step=.003
        req.jump_threshold=2.;req.revolute_jump_threshold=.15;req.avoid_collisions=True
        req.max_velocity_scaling_factor=.05;req.max_acceleration_scaling_factor=.05
        for w in waypoints:
            p=Pose();p.position.x,p.position.y,p.position.z=w['position'];p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=w['orientation'];req.waypoints.append(p)
        self.future=self.plan.call_async(req);self.phase=phase;self.deadline=time.monotonic()+10

    def next_segment(self):
        self.settled_since=None
        self.phase='segment_settle';self.deadline=time.monotonic()+5

    def accepted(self,future):
        self.submitting=False
        try:
            handle=future.result()
            if not handle.accepted:
                self.robot.sequence_motion_busy=False
                if not self.cancelled:self.fail('trajectory rejected')
                return
            self.handle=handle
            handle.get_result_async().add_done_callback(self.result)
            if self.cancelled or self.g.generation!=self.generation:handle.cancel_goal_async()
        except Exception:self.submitting=True;self.fail('trajectory submission outcome unknown')

    def result(self,future):
        self.handle=None;self.robot.sequence_motion_busy=False
        if self.cancelled or self.g.generation!=self.generation:return
        try:
            result=future.result()
            if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:raise ValueError('trajectory failed')
            self.phase='arrival';self.settled_since=None;self.deadline=time.monotonic()+5
        except Exception as exc:self.fail(str(exc))

    def arrived(self):
        stage=self.waypoints[self.index]['stage']
        if stage=='grasp':
            self.phase='closing';self.deadline=time.monotonic()+12
            self.g.set_state('active','grasp_closing','已到抓取点，闭爪')
            self.g.issue(lambda cb:self.robot.grip(True,cb),self.closed)
        elif stage=='lift':
            self.phase='verify';self.verify_frames=0;self.verify_on_table=False
            self.verify_since=time.monotonic();self.deadline=self.verify_since+self.verify_sec+3
            self.g.set_state('active','grasp_verify','检查夹爪与相机：是否抓到小球')
        elif stage=='place':self.release('grasp_placing','放回小球，张开夹爪')
        elif stage=='home':self.complete()
        else:self.index+=1;self.next_segment()

    def closed(self):
        if self.cancelled:return
        self.index+=1;self.next_segment()

    def release(self,phase,message):
        self.phase='releasing';self.deadline=time.monotonic()+12
        self.g.set_state('active',phase,message)
        self.g.issue(lambda cb:self.robot.grip(False,cb),self.released)

    def released(self):
        if self.cancelled:return
        self.index+=1;self.next_segment()

    def wrapup_tick(self,now):
        if self.phase=='verify':
            if now-self.verify_since<self.verify_sec:return
            self.success,reason=grasp_evidence(self.finger(),self.verify_frames,self.verify_on_table,self.empty_finger,self.min_finger)
            self.g.outcome=('simulated_grasp' if self.robot.mock else 'grasp_success') if self.success else 'grasp_missed'
            self.waypoints=self.waypoints[:self.index+1]+wrapup_waypoints(self.waypoints,self.home,self.success)
            if self.success:
                self.phase='show';self.show_until=now+self.show_sec;self.deadline=self.show_until+3
                self.g.set_state('active','grasp_show','抓到啦！'+reason)
            else:self.release('grasp_missed','没抓到：'+reason+'；张开夹爪后收回')
        elif self.phase=='show' and now>=self.show_until:
            self.index+=1;self.next_segment()

    def complete(self):
        # Back at the sequence start view: hand over to impedance, react, then go home.
        self.running=False;self.phase='complete';self.frozen=None;self.stable=False;self.gate.reset()
        success=self.success
        def react():
            if success:self.g.return_home();return
            start=list(self.robot.current);tilt=list(start);tilt[5]+=-.10 if start[5]>0 else .10
            self.g.set_state('transition','puzzled','没抓到，歪头看看小球')
            self.g.move([tilt,start],self.g.return_home)
        self.g.set_state('transition','grasp_wrapup','回到观察位，切回阻抗保持')
        self.g.impedance(react)

    def tick(self):
        if not self.running:return
        now=time.monotonic()
        try:
            if now>self.deadline:raise ValueError(self.phase+' timeout')
            if self.index<5 and not self.target_valid():raise ValueError('目标移动或定位丢失')
            if self.robot.mock:
                self.mock_tick(now);return
            if not self.robot.fresh() or now-self.feedback_at>.25:raise ValueError('joint/controller feedback stale')
            if self.feedback[1]>.06:raise ValueError('position following error')
            if self.g.count_publishers('/'+self.robot.modes.pos+'/joint_trajectory')>int(self.robot.position_pub is not None):raise ValueError('another position publisher')
            if self.phase in ('verify','show'):
                self.wrapup_tick(now);return
            if self.phase in ('settle','segment_settle'):
                if not self.robot.stationary():return
                if self.settled_since is None:self.settled_since=now
                if now-self.settled_since<.5:return
                self.future=self.robot.modes.list.call_async(ListControllers.Request())
                self.phase='guard_all' if self.phase=='settle' else 'guard_segment';return
            if self.future is not None:
                if not self.future.done():return
                result=self.future.result();self.future=None
                if self.phase.startswith('guard_'):
                    reason=self.robot.modes.guard(result.controller)
                    if reason or observed(result.controller,self.robot.motion.prefix)!='position':raise ValueError(reason or 'position controller required')
                    if self.phase=='guard_all':
                        if not self.scene.service_is_ready():raise ValueError('planning scene unavailable')
                        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                        self.future=self.scene.call_async(req);self.phase='scene';return
                    self.request_plan([self.waypoints[self.index]],'segment_plan');return
                if self.phase=='scene':
                    objects=result.scene.world.collision_objects
                    if not self.table_id or not any(o.id==self.table_id and (o.primitives or o.meshes) for o in objects):raise ValueError('缺少桌面碰撞模型')
                    tcp=self.buffer.lookup_transform('base_link','omni_picker_tcp',Time()).transform
                    q=[tcp.rotation.x,tcp.rotation.y,tcp.rotation.z,tcp.rotation.w]
                    self.home=dict(stage='home',position=[tcp.translation.x,tcp.translation.y,tcp.translation.z],orientation=q)
                    self.waypoints=preview_waypoints(self.frozen,self.target['radius'],orientation=q)
                    self.request_plan(self.waypoints,'full_plan');return
                if self.phase in ('full_plan','segment_plan'):
                    if result.error_code.val!=1 or not math.isfinite(result.fraction) or result.fraction<.999:raise ValueError('抓取路径不完整，未执行剩余路径')
                    points=timed_points(result.solution.joint_trajectory,self.robot.current)
                    if self.phase=='full_plan':
                        self.phase='opening';self.deadline=now+12
                        self.g.issue(lambda cb:self.robot.grip(False,cb),self.next_segment);return
                    if not self.action.server_is_ready():raise ValueError('position action unavailable')
                    goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=JOINTS;goal.trajectory.points=points
                    self.goal=list(points[-1].positions);self.phase='moving';self.submitting=True;self.robot.sequence_motion_busy=True
                    self.deadline=now+points[-1].time_from_start.sec+points[-1].time_from_start.nanosec*1e-9+5
                    self.g.set_state('active','grasp_'+self.waypoints[self.index]['stage'],'执行抓球阶段：'+self.waypoints[self.index]['stage'])
                    self.action.send_goal_async(goal).add_done_callback(self.accepted);return
            if self.phase=='arrival':
                arrived=self.robot.stationary() and max(abs(a-b) for a,b in zip(self.robot.current,self.goal))<.01
                self.settled_since=(self.settled_since or now) if arrived else None
                if self.settled_since is not None and now-self.settled_since>.3:self.arrived()
        except Exception as exc:self.fail(str(exc))

    def mock_tick(self,now):
        # Mock exercises the same stages, close-before-lift and verify/wrap-up ordering, without ROS planning/hardware.
        if self.phase=='settle':
            self.waypoints=preview_waypoints(self.frozen,self.target['radius'])
            self.home=dict(stage='home',position=[.3,0.,.3],orientation=[1.,0.,0.,0.])
            self.phase='opening';self.deadline=now+5
            self.g.issue(lambda cb:self.robot.grip(False,cb),self.next_segment)
        elif self.phase in ('verify','show'):self.wrapup_tick(now)
        elif self.phase=='segment_settle':
            self.phase='mock_moving';self.deadline=now+5
            self.g.set_state('active','grasp_'+self.waypoints[self.index]['stage'],'模拟 '+self.waypoints[self.index]['stage'])
            self.g.move([self.robot.current],self.arrived)

    def destroy(self):self.action.destroy()
