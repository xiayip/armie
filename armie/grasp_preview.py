"""Read-only RGB-D localization and MoveIt planning. No motor/action command client."""
import json,math,time
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from tf2_ros import Buffer,TransformListener,TransformException
from .ball_grasp_geometry import locate_ball,WorldStillGate,preview_waypoints,rotation


class GraspPreview(Node):
    def __init__(self):
        super().__init__('armie_grasp_preview')
        self.declare_parameter('output','/tmp/armie-grasp-preview.json')
        self.output=self.get_parameter('output').value
        self.depth=deque(maxlen=12);self.info=None;self.joints=None;self.state={};self.gate=WorldStillGate()
        self.last_stamp=0.;self.last_report=0.;self.future=None;self.pending=None;self.last_planned=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(String,'/armie/grasp_preview',1)
        self.client=self.create_client(GetCartesianPath,'/compute_cartesian_path')
        self.create_subscription(Image,'/camera/depth/image_raw',lambda m:self.depth.append(m),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/color/camera_info',lambda m:setattr(self,'info',m),qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',m),qos_profile_sensor_data)
        self.create_subscription(String,'/armie/state',lambda m:setattr(self,'state',json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/armie/observation',self.observe,1)
        self.create_timer(.05,self.tick)
        self.target_at=0.
        self.last_invalid_reason=None

    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

    def report(self,**values):
        record=dict(preview_only=True,execution_allowed=False,tcp_calibrated=False,
                    frame='base_link',**values)
        self.pub.publish(String(data=json.dumps(record)))
        if self.output:
            path=Path(self.output);path.parent.mkdir(parents=True,exist_ok=True)
            tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(record,indent=2));tmp.replace(path)
            if values.get('status') in ('preview_ready','path_incomplete'):
                path.with_name(path.stem+'-planned.json').write_text(json.dumps(record,indent=2))
        self.last_report=time.monotonic()

    def invalidate(self,reason):
        if reason!=self.last_invalid_reason:self.get_logger().info(reason);self.last_invalid_reason=reason
        self.gate.reset();self.pending=None;self.last_planned=None
        if time.monotonic()-self.last_report>.5:self.report(status='waiting',reason=reason)

    def observe(self,msg):
        try:
            d=json.loads(msg.data);stamp=d['stamp'];now=self.get_clock().now().nanoseconds*1e-9
            if self.state.get('state')!='paused':raise ValueError('preview requires paused robot')
            if d.get('source')!='camera' or not d.get('valid') or not d.get('ball'):raise ValueError('ball not detected')
            if stamp<=self.last_stamp:return
            self.last_stamp=stamp
            if not 0<=now-stamp<=.15:raise ValueError('stale ball observation')
            if not self.depth or self.info is None or self.joints is None:raise ValueError('waiting for depth/intrinsics/joints')
            if now-self.stamp(self.joints)>.2:raise ValueError('stale joints')
            image=min(self.depth,key=lambda m:abs(self.stamp(m)-stamp))
            if abs(self.stamp(image)-stamp)>.05:return  # Wait for the next synchronized pair; watchdog bounds the gap.
            if (image.width,image.height)!=(self.info.width,self.info.height) or image.header.frame_id!=self.info.header.frame_id:
                raise ValueError('depth must be registered to color intrinsics')
            dtype={'16UC1':np.dtype('>u2' if image.is_bigendian else '<u2'),'32FC1':np.dtype('>f4' if image.is_bigendian else '<f4')}.get(image.encoding)
            if dtype is None:raise ValueError('unsupported depth encoding')
            depth=np.frombuffer(image.data,dtype).reshape(image.height,image.step//dtype.itemsize)[:,:image.width].astype(float)
            if image.encoding=='16UC1':depth*=.001
            tr=self.buffer.lookup_transform('base_link',image.header.frame_id,Time.from_msg(image.header.stamp)).transform
            target=locate_ball(depth,self.info.k,d['ball'],[tr.translation.x,tr.translation.y,tr.translation.z],
                [tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
            self.target_at=time.monotonic()
            stable=self.gate.update(target['center'],stamp)
            if self.pending and np.linalg.norm(np.array(target['center'])-self.pending['target']['center'])>.01:self.pending=None
            if not stable:
                if self.last_planned is not None and np.linalg.norm(np.array(target['center'])-self.last_planned)>.01:self.last_planned=None
                if time.monotonic()-self.last_report>.5:self.report(status='measuring_stillness',target=target,still_seconds=stamp-self.gate.since)
                return
            if self.future is not None or self.last_planned is not None:return
            if not self.client.service_is_ready():raise ValueError('MoveIt planning service unavailable')
            tcp=self.buffer.lookup_transform('base_link','omni_picker_tcp',Time.from_msg(image.header.stamp)).transform
            rot=rotation([tcp.rotation.x,tcp.rotation.y,tcp.rotation.z,tcp.rotation.w]);yaw=math.atan2(rot[1,0],rot[0,0])
            waypoints=preview_waypoints(target['center'],target['radius'],yaw,[tcp.rotation.x,tcp.rotation.y,tcp.rotation.z,tcp.rotation.w])
            request=GetCartesianPath.Request();request.header.frame_id='base_link'
            request.start_state.joint_state=self.joints;request.start_state.is_diff=True
            request.group_name='zephyr_arm_tcp';request.link_name='omni_picker_tcp'
            request.max_step=.005;request.jump_threshold=2.;request.revolute_jump_threshold=.25
            request.avoid_collisions=True;request.max_velocity_scaling_factor=.1;request.max_acceleration_scaling_factor=.1
            for w in waypoints:
                p=Pose();p.position.x,p.position.y,p.position.z=w['position'];p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=w['orientation']
                request.waypoints.append(p)
            self.pending=dict(target=target,waypoints=waypoints,still_seconds=stamp-self.gate.since)
            self.future=self.client.call_async(request);self.future_started=time.monotonic()
            self.report(status='planning',**self.pending)
        except TransformException:
            # TF often arrives a few ms after RGB-D. Never substitute latest TF;
            # skip this pair, with a 200 ms freshness watchdog below.
            return
        except Exception as exc:self.invalidate(str(exc))

    def tick(self):
        if self.target_at and time.monotonic()-self.target_at>.2:self.invalidate('target depth lost')
        if self.future is None:return
        if not self.future.done():
            if time.monotonic()-self.future_started>10:
                self.future.cancel();self.future=None;self.invalidate('planning timeout')
            return
        future,self.future=self.future,None
        if self.pending is None:return
        context,self.pending=self.pending,None
        try:
            result=future.result();trajectory=result.solution.joint_trajectory
            rows=[dict(t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9,q=list(p.positions),v=list(p.velocities)) for p in trajectory.points]
            complete=result.error_code.val==1 and result.fraction>=.999 and len(rows)>1
            self.last_planned=np.array(context['target']['center'])
            self.report(status='preview_ready' if complete else 'path_incomplete',**context,
                planning_fraction=result.fraction,planning_error=result.error_code.val,joint_names=list(trajectory.joint_names),trajectory=rows,
                limitations=['TCP offset requires physical calibration','Ball center is a fitted sphere estimate','Table and surrounding objects are not yet collision-mapped','Gripper close and lift are preview events, not executed'])
        except Exception as exc:self.invalidate(str(exc))


def main():
    rclpy.init();node=GraspPreview()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
