"""Operator-confirmed pivot calibration; reads poses and never commands hardware."""
import json,math,time
from collections import deque
from pathlib import Path
import numpy as np
from .ball_grasp_geometry import rotation


def solve_pivot(samples):
    if len(samples)<6:raise ValueError('need at least six manually aligned poses')
    matrices=[];translations=[]
    for s in samples:
        r=rotation(s['rotation']);t=np.asarray(s['translation'],dtype=float)
        if t.shape!=(3,) or not np.isfinite(t).all():raise ValueError('invalid translation')
        matrices.append(r);translations.append(t)
    # R_i * tool_offset + translation_i = the same fixed point in base frame.
    a=np.vstack([np.hstack((r,-np.eye(3))) for r in matrices]);b=-np.concatenate(translations)
    result,_,rank,singular=np.linalg.lstsq(a,b,rcond=None)
    condition=float(singular[0]/singular[-1]) if singular[-1]>0 else math.inf
    if rank!=6 or condition>80:raise ValueError('insufficient orientation diversity; tilt about two different axes')
    offset=result[:3];point=result[3:];errors=np.array([r@offset+t-point for r,t in zip(matrices,translations)])
    distances=np.linalg.norm(errors,axis=1);rms=float(np.sqrt(np.mean(distances**2)))
    if rms>.003 or max(distances)>.006:raise ValueError('alignment residual too large; fixed point or tip alignment moved')
    if not .06<np.linalg.norm(offset)<.30:raise ValueError('implausible tool offset; check frame and alignment point')
    return dict(parent_frame='omni_picker_connector',offset_xyz_m=offset.tolist(),fixed_point_base_m=point.tolist(),
                samples=len(samples),rms_m=rms,max_error_m=float(max(distances)),condition_number=condition,
                candidate_only=True,applied=False,orientation_calibrated=False)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer,TransformListener
    class Calibration(Node):
        def __init__(self):
            super().__init__('armie_tcp_calibration')
            self.declare_parameter('output','/tmp/armie-tcp-calibration.json')
            self.output=Path(self.get_parameter('output').value)
            self.samples=[];self.window=deque();self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.gripper=None
            self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
            self.create_service(Trigger,'/armie/tcp_calibration/capture',self.capture)
            self.create_service(Trigger,'/armie/tcp_calibration/solve',self.solve)
            self.create_service(Trigger,'/armie/tcp_calibration/undo',self.undo)
            if self.output.exists():
                stored=json.loads(self.output.read_text());self.samples=stored.get('samples',[])
        def joints(self,m):
            try:
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                q=[m.position[m.name.index('joint'+str(i))] for i in range(1,7)]
                grip=m.position[m.name.index('omni_picker_finger_joint')]
                if not all(math.isfinite(x) for x in [stamp,*q,grip]):raise ValueError()
                if self.window and (stamp<=self.window[-1][0] or stamp-self.window[-1][0]>.15):self.window.clear()
                self.window.append((stamp,q,grip))
                while len(self.window)>1 and stamp-self.window[1][0]>.6:self.window.popleft()
            except (ValueError,IndexError):self.window.clear()
        def save(self,result=None):
            self.output.parent.mkdir(parents=True,exist_ok=True)
            tmp=self.output.with_suffix('.tmp');tmp.write_text(json.dumps(dict(samples=self.samples,result=result),indent=2));tmp.replace(self.output)
        def capture(self,request,response):
            try:
                now=self.get_clock().now().nanoseconds*1e-9
                if len(self.window)<10 or self.window[-1][0]-self.window[0][0]<.5 or now-self.window[-1][0]>.15:
                    raise ValueError('need fresh, stationary joint feedback for 0.5 seconds')
                q=np.array([x[1] for x in self.window]);grips=[x[2] for x in self.window]
                if np.max(np.ptp(q,axis=0))>.002 or max(grips)-min(grips)>.005:raise ValueError('arm or gripper still moving')
                if self.samples and abs(grips[-1]-self.samples[0]['gripper_position'])>.01:raise ValueError('keep identical gripper opening throughout calibration')
                tr=self.buffer.lookup_transform('base_link','omni_picker_connector',Time())
                stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
                if not 0<=now-stamp<.15:raise ValueError('stale tool transform')
                t=tr.transform.translation;r=tr.transform.rotation
                sample=dict(stamp=stamp,translation=[t.x,t.y,t.z],rotation=[r.x,r.y,r.z,r.w],gripper_position=grips[-1],joints=q[-1].tolist())
                if self.samples:
                    nr=rotation(sample['rotation'])
                    angles=[math.acos(np.clip((np.trace(rotation(s['rotation']).T@nr)-1)/2,-1,1)) for s in self.samples]
                    if min(angles)<math.radians(8):raise ValueError('pose repeats existing orientation; re-align from another angle')
                self.samples.append(sample);self.save();response.success=True;response.message=f'captured aligned pose {len(self.samples)}'
            except Exception as exc:response.message=str(exc)
            return response
        def solve(self,request,response):
            try:
                result=solve_pivot(self.samples);result['gripper_position']=self.samples[0]['gripper_position'];self.save(result)
                response.success=True;response.message=json.dumps(result)
            except Exception as exc:response.message=str(exc)
            return response
        def undo(self,request,response):
            if self.samples:self.samples.pop();self.save();response.success=True
            response.message=f'{len(self.samples)} samples remain';return response
    rclpy.init();node=Calibration()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
