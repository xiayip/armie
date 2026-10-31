"""Armie v0.2 four-mode game. Numeric gestures select modes only while idle."""
import json
import math
import time
from pathlib import Path

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ghost_game_orchestrator.face_tracking import make_face_sample, face_matches_track, servo_target, scan_target
from .common import STATE_QOS, finite, param, publish, run
from .game_core import DigitGate, StillGate, pose
from .motion import LOWER, UPPER
from .robot import Robot
from .ball_prediction import BallPrediction
from .ball_sequence import BallSequence

MODES={1:'soft',2:'ball',3:'dance',4:'face'}


class Game(Node):
    def __init__(self):
        super().__init__('armie_game')
        self.robot=Robot(self)
        self.motion=self.robot  # Common shutdown waits for all owned actions/transitions.
        self.front=json.loads(param(self,'front_pose_json','[]'))
        self.table=json.loads(param(self,'table_pose_json','[]'))
        self.grasp=json.loads(param(self,'grasp_path_json','[]'))
        self.calibration_file=param(self,'calibration_file','')
        if not self.robot.mock and self.calibration_file and Path(self.calibration_file).is_file():
            saved=json.loads(Path(self.calibration_file).read_text())
            if saved.get('source')!='measured':raise ValueError('calibration file must contain measured poses')
            self.front=saved.get('front',self.front)
            self.table=saved.get('table',self.table)
        if self.front:self.front=pose(self.front)
        if self.table:self.table=pose(self.table)
        if self.grasp:
            if len(self.grasp)!=3:raise ValueError('grasp_path_json requires [pregrasp,grasp,lift]')
            self.grasp=[pose(p) for p in self.grasp]
        if self.robot.mock:
            self.front=self.front or self.robot.current
            self.table=self.table or [0.,.6,.8,.2,0.,0.]
        self.grasp_enabled=param(self,'grasp_enabled',False)
        if self.grasp_enabled and self.robot.gripper_open_only:
            raise ValueError('grasp_enabled conflicts with gripper_open_only')
        self.dance_amplitude=finite(param(self,'dance_amplitude_scale',1.),.25,1.,'dance amplitude')
        self.dance_speed=finite(param(self,'dance_speed_scale',1.),.5,1.6,'dance speed')
        self.pose_transition_speed=finite(param(self,'pose_transition_speed_scale',1.),1.,4.,'pose transition speed')
        self.ball_controller=param(self,'ball_controller','impedance')
        if self.ball_controller not in ('impedance','position'):raise ValueError('unsupported ball controller')
        self.ball_tracking_gain=finite(param(self,'ball_tracking_gain',.5),.1,2.,'ball tracking gain')
        self.face_duration=finite(param(self,'face_duration_sec',18.),1.,120.,'face duration')
        self.ball_follow_only=param(self,'ball_follow_only',False)
        self.ball_duration=finite(param(self,'ball_duration_sec',25.),0.,120.,'ball duration')
        self.ball_wait_for_target=param(self,'ball_wait_for_target',False)
        self.ball_still_sec=finite(param(self,'ball_still_sec',.8),.5,10.,'ball still dwell')
        self.ball_prediction_enabled=param(self,'ball_prediction_enabled',False)
        self.ball_prediction=BallPrediction(lead=finite(param(self,'ball_prediction_lead_sec',.04),0.,.08,'prediction lead'))
        self.yaw_direction=finite(param(self,'yaw_direction',-1.),-1.,1.,'yaw direction')
        self.pitch_direction=finite(param(self,'pitch_direction',-1.),-1.,1.,'pitch direction')
        self.servo_commissioned=param(self,'servo_commissioned',False)
        self.ball_servo_commissioned=param(self,'ball_servo_commissioned',self.servo_commissioned)
        self.dock_roi=json.loads(param(self,'dock_roi_json','[-0.2,-0.2,0.2,0.2]'))
        if len(self.dock_roi)!=4 or not all(isinstance(v,(int,float)) and math.isfinite(v) and -1<=v<=1 for v in self.dock_roi) or self.dock_roi[0]>=self.dock_roi[2] or self.dock_roi[1]>=self.dock_roi[3]:raise ValueError('invalid dock ROI')
        self.state='idle' if self.robot.mock else 'paused'
        self.mode,self.phase,self.message='','waiting','朝人静止待机' if self.robot.mock else '记录朝人姿态或配置姿态后点击准备'
        self.round_id,self.generation=0,0
        self.sample,self.previous_face,self.last_seen=None,None,0.
        self.anchor=None
        self.started,self.phase_at,self.last_tick=time.monotonic(),time.monotonic(),time.monotonic()
        self.gate=DigitGate()
        self.still=StillGate(dwell=self.ball_still_sec)
        self.pending,self.pending_deadline=None,0.
        self.released=False
        self.release_since=None
        self.finish_deadline=0.
        self.outcome=''
        self.sensor_session,self.sensor_sequence=None,-1
        self.pub=self.create_publisher(String,'/armie/state',STATE_QOS)
        self.tracking_pub=self.create_publisher(String,'/armie/tracking_status',1)
        self.last_tracking_report=0.
        self.sub=self.create_subscription(String,'/armie/command',lambda m:self.dispatch(m.data.strip().lower()),10)
        self.sensor_sub=self.create_subscription(String,'/armie/observation',self.observation,1)
        self.command_services=[]
        for name in ['finish','release','stop','ready','ready_table','record_front','record_table']:
            self.command_services.append(self.create_service(Trigger,'/armie/'+name,lambda req,res,n=name:self.service(n,res)))
        self.ball_sequence=BallSequence(self)
        self.timer=self.create_timer(.025,self.tick)
        self.heartbeat=self.create_timer(.2,self.publish_state)
        self.publish_state()

    def set_state(self,state,phase,message):
        self.state,self.phase,self.message=state,phase,message
        self.phase_at=time.monotonic()
        self.publish_state()

    def publish_state(self):
        publish(self.pub,dict(schema_version=2,state=self.state,mode=self.mode,phase=self.phase,
                             message=self.message,round_id=self.round_id,outcome=self.outcome,
                             mock=self.robot.mock,motion_enabled=self.robot.motion.enabled,
                             motion_busy=self.robot.busy or self.ball_sequence.busy,controller_mode=self.robot.modes.mode,
                             front_pose=self.front,table_pose=self.table,
                             gesture_enabled=self.state=='idle'))

    def service(self,command,response):
        response.success,response.message=self.command(command)
        return response

    def dispatch(self,command):
        accepted,reason=self.command(command)
        if not accepted:
            self.message=reason
            self.publish_state()
        return accepted,reason

    def callback(self,next_step):
        generation=self.generation
        def done(ok,message):
            if generation!=self.generation:return
            if not ok:self.pause(message)
            else:next_step()
        return done

    def issue(self,operation,next_step):
        ok,reason=operation(self.callback(next_step))
        if not ok:self.pause(reason)
        return ok

    def wait_settled(self,next_step):
        self.pending,self.pending_deadline=next_step,time.monotonic()+5.
        self.set_state('transition','settling','等待控制器参考与实测姿态稳定')

    def impedance(self,next_step):
        self.robot.hold()
        self.issue(lambda cb:self.robot.mode('impedance',cb),lambda:self.wait_settled(next_step))

    def move(self,targets,next_step,speed=.12,minimum=1.):
        return self.issue(lambda cb:self.robot.move(targets,cb,speed,minimum),next_step)

    def move_to_pose(self,targets,next_step):
        return self.move(targets,next_step,.12*self.pose_transition_speed,1./self.pose_transition_speed)

    def command(self,command):
        if command=='stop':
            self.pause('互动已暂停；仅取消自己的动作，不自动回正')
            return True,'stop requested'
        if command=='finish':
            # Like ghost_game's return_home: accepted from any state, preempts the
            # current round (only our own goals are cancelled), then glides home.
            if not self.front:return False,'front pose not configured'
            if self.state=='finishing' or self.phase in ('finish_opening','returning'):return False,'already returning home'
            self.ball_sequence.stop()
            self.generation+=1
            self.pending=None
            self.released=False
            self.release_since=None
            self.finish_deadline=time.monotonic()+10.
            self.robot.fault=''
            self.robot.stop()
            self.set_state('finishing','waiting_stop','等待当前动作停止，然后回正')
            return True,'returning home; soft mode also requires release confirmation'
        if self.ball_sequence.busy:return False,'grasp sequence busy or cancellation pending'
        if command=='release':
            if self.state!='finishing' or self.mode!='soft':return False,'release confirmation is only for soft-mode exit'
            self.released=True
            self.release_since=None
            self.finish_deadline=time.monotonic()+10.
            return True,'release confirmed; waiting for stationary joints'
        if command in ('record_front','record_table'):
            if self.robot.busy or self.robot.servo or not self.robot.stationary():return False,'requires stationary joints and no pending motion'
            if self.state not in ('idle','paused'):return False,'finish the current mode first'
            current=pose(self.robot.current)
            if command=='record_front':self.front=current
            else:self.table=current
            if self.calibration_file and not self.robot.mock:
                path=Path(self.calibration_file)
                path.parent.mkdir(parents=True,exist_ok=True)
                temporary=path.with_suffix('.tmp')
                temporary.write_text(json.dumps({'source':'measured','front':self.front,'table':self.table}))
                temporary.replace(path)
            self.publish_state()
            return True,json.dumps(current)
        if command=='preview_ball':
            if self.state!='paused' or self.robot.busy:return False,'pause before preview'
            self.mode='ball'
            self.set_state('paused','preview','仅预览小球识别，不发送运动')
            return True,'ball preview enabled'
        if command=='ready_table':
            if self.robot.busy or self.state not in ('idle','paused'):return False,'robot busy'
            if not self.table:return False,'table pose not configured'
            self.generation+=1
            self.robot.fault=''
            self.mode=''
            self.set_state('transition','preparing_table','准备已记录的桌面视角')
            self.impedance(lambda:self.move_to_pose([self.table],lambda:self.set_state('idle','waiting','桌面视角就绪；手势 2 跟踪小球')))
            return True,'preparing table view'
        if command=='ready':
            if self.robot.busy or self.state not in ('idle','paused'):return False,'robot busy'
            if not self.front:return False,'record or configure the person-facing pose first'
            self.generation+=1
            self.robot.fault=''
            self.mode=''
            self.set_state('transition','preparing','准备朝人待机')
            self.impedance(self.return_home)
            return True,'preparing'
        try:number=int(command)
        except ValueError:number=next((n for n,m in MODES.items() if m==command),0)
        if number not in MODES:return False,'unknown command'
        if self.state!='idle' or self.robot.busy:return False,'mode selection is only available while idle'
        if not self.front:return False,'front pose not configured'
        commissioned=self.ball_servo_commissioned if number==2 else self.servo_commissioned
        if number in (2,4) and not self.robot.mock and not commissioned:return False,'commission camera directions before tracking'
        if number==2 and not self.table:return False,'record the table observation pose first'
        if not self.robot.mock and not self.robot.motion.enabled:return False,'motion_enabled is false'
        self.generation+=1
        self.round_id+=1
        self.mode,self.outcome=MODES[number],''
        self.gate.update(0,time.monotonic(),enabled=False)
        self.sample,self.previous_face,self.anchor=None,None,None
        self.still.reset()
        self.set_state('transition','entering','进入'+self.mode)
        if self.mode=='soft':
            self.issue(lambda cb:self.robot.mode('damping',cb),lambda:self.set_state('active','dragging','可柔顺拖动；结束后松手并点击确认松手'))
        else:
            self.impedance(self.start_mode)
        return True,'accepted'

    def start_mode(self):
        if self.mode=='dance':
            self.set_state('active','dancing','水草舞：整臂连续起伏')
            self.issue(lambda cb:self.robot.dance(self.front,cb,self.dance_amplitude,self.dance_speed),self.return_home)
        else:
            self.set_state('transition','positioning','转向观察姿态')
            if self.mode=='ball' and (self.robot.gripper_open_only or self.ball_sequence.enabled):
                self.set_state('transition','opening_gripper','先张开夹爪，减少腕部相机遮挡')
                self.issue(lambda cb:self.robot.grip(False,cb),lambda:self.move_to_pose([self.table],self.begin_tracking))
            elif self.mode=='ball':
                self.move_to_pose([self.table],self.begin_tracking)
            else:
                self.move([self.front],self.begin_tracking)

    def begin_tracking(self):
        self.issue(lambda cb:self.robot.mode('position' if self.mode=='ball' and self.ball_controller=='position' else 'servo',cb),self.tracking_started)

    def tracking_started(self):
        self.robot.set_tracking_profile(self.mode=='ball')
        self.anchor=self.robot.current
        self.started=self.last_seen=time.monotonic()
        self.sample,self.previous_face=None,None
        self.ball_prediction.reset()
        self.still.reset()
        self.ball_sequence.gate.reset();self.ball_sequence.stable=False;self.ball_sequence.frozen=None
        self.set_state('active','watching' if self.mode=='ball' else 'finding','观察小球' if self.mode=='ball' else '寻找人脸')

    def observation(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict) or value.get('schema_version')!=1:return
            if value.get('source') not in ('mock','camera','external'):return
            if not self.robot.mock and value.get('source')=='mock':return
            stamp=finite(value['stamp'],0.,1e12,'stamp')
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0<=age<=.3:return
            session=value['session']
            sequence=value['sequence']
            if not isinstance(session,str) or type(sequence) is not int:return
            if session==self.sensor_session and sequence<=self.sensor_sequence:return
            if session!=self.sensor_session:self.ball_prediction.reset()
            self.sensor_session,self.sensor_sequence=session,sequence
            now=time.monotonic()
            valid=value.get('valid') is True
            digit=value.get('digit',0)
            if type(digit) is not int:return
            if value.get('round_id')!=self.round_id:return
            self.ball_sequence.observe(value)
            selected=self.gate.update(digit,now,self.state=='idle',valid)
            if selected is not None:self.dispatch(str(selected))
            if self.state!='active' or self.mode not in ('ball','face'):return
            sample=value.get(self.mode)
            if not valid or not isinstance(sample,dict):
                if self.mode=='ball':
                    self.sample=None
                    self.ball_prediction.reset()
                return
            x=finite(sample['x'],-1.,1.,'x'); y=finite(sample['y'],-1.,1.,'y')
            w=finite(sample['width'],.001,2.,'width');h=finite(sample['height'],.001,2.,'height')
            score=finite(sample['score'],0.,1.,'score')
            if score<.7 or (self.mode=='ball' and age>.12):
                if self.mode=='ball':
                    self.sample=None
                    self.ball_prediction.reset()
                return
            if self.mode=='face' and not .005<=w*h/4<=.55:return
            if self.mode=='ball' and not self.ball_prediction.update(stamp,x,y,score):return
            face=make_face_sample(now-age if self.mode=='ball' else now,x+1,y+1,w,h,2,2,score)
            if self.mode=='face' and self.previous_face and not face_matches_track(self.previous_face,face,.3,.8):return
            self.sample=face
            self.previous_face=face if self.mode=='face' else None
            self.last_seen=now
        except (ValueError,TypeError,KeyError):return

    def tracking_tick(self,now,dt):
        if self.mode=='ball' and self.ball_sequence.enabled and self.ball_sequence.stable and self.ball_sequence.target_valid():
            self.ball_sequence.begin();return
        duration=self.face_duration if self.mode=='face' else self.ball_duration
        if duration>0 and now-self.started>duration:
            self.command('finish');return
        if self.sample is None or now-self.sample.received_at>(.12 if self.mode=='ball' else .25):
            if self.mode=='ball':self.ball_prediction.reset()
            self.still.reset()
            self.robot.track(self.robot.reference,dt)
            self.report_tracking(now,None)
            if self.mode=='face' and self.previous_face is None:
                if now-self.started<4.:
                    target=scan_target(self.anchor,LOWER,UPPER,4,3,.10*math.sin((now-self.started)*.8),0.)
                    self.robot.track(target,dt)
            if self.mode=='ball' and self.ball_wait_for_target:
                self.phase='waiting_target'
                self.message='小球暂时丢失，减速保持并等待重新识别'
                return
            if now-self.last_seen>4.:
                self.outcome='target_lost'
                self.command('finish')
            return
        if self.mode=='ball' and self.phase=='waiting_target':
            self.phase='watching';self.message='重新识别小球，恢复跟踪'
        sample=self.sample
        error=(sample.error_x,sample.error_y)
        if self.mode=='ball' and self.ball_prediction_enabled:
            predicted=self.ball_prediction.predict(self.get_clock().now().nanoseconds*1e-9)
            if predicted is not None:error=predicted
        gain=self.ball_tracking_gain if self.mode=='ball' else .5
        target=servo_target(self.robot.reference,self.anchor,LOWER,UPPER,4,3,
            *error,gain,gain,self.robot.tracking_speed,dt,self.yaw_direction,self.pitch_direction,.35,.25,.08)
        self.robot.track(target,dt)
        self.report_tracking(now,error)
        if self.mode=='face':
            self.phase='following'
        elif not self.ball_sequence.enabled and not self.ball_follow_only and now-self.started>2. and self.still.update((sample.error_x,sample.error_y),self.robot.current,now):
            self.robot.hold()
            self.set_state('transition','return_to_dock_view','小球停稳，恢复已标定的桌面视角')
            self.impedance(lambda:self.move([self.table],self.dock_started))

    def report_tracking(self,now,error):
        if now-self.last_tracking_report<.1:return
        self.last_tracking_report=now
        publish(self.tracking_pub,dict(mode=self.mode,target_valid=error is not None,
            image_error=list(error) if error is not None else None,
            sample_age=now-self.sample.received_at if self.sample is not None else None,
            reference=self.robot.reference,anchor=self.anchor,
            commanded_velocity=self.robot.tracking_velocity,
            speed_limit=self.robot.tracking_speed,acceleration_limit=self.robot.tracking_acceleration))

    def dock_started(self):
        self.sample=None
        self.still.reset()
        self.set_state('active','dock_check','确认小球在固定抓取区内停稳')

    def dock_tick(self,now):
        if now-self.phase_at>5.:
            self.outcome='outside_dock_or_lost'
            self.command('finish');return
        s=self.sample
        if s is None or now-s.received_at>.25 or not self.robot.stationary():
            self.still.reset();return
        x0,y0,x1,y1=self.dock_roi
        if not (x0<=s.error_x<=x1 and y0<=s.error_y<=y1):
            self.still.reset();return
        if not self.still.update((s.error_x,s.error_y),self.robot.current,now):return
        self.set_state('active','attempt','尝试抓球')
        if not self.grasp_enabled or not self.grasp or (not self.robot.mock and not self.robot.gripper_enabled):
            target=list(self.table);target[5]+= .10
            self.outcome='curious_attempt_only'
            self.move([pose(target),self.table],self.return_home)
        else:
            self.issue(lambda cb:self.robot.grip(False,cb),lambda:self.move(self.grasp[:2],self.close_grip))

    def close_grip(self):
        self.issue(lambda cb:self.robot.grip(True,cb),self.show_attempt)

    def show_attempt(self):
        self.outcome='simulated_grasp' if self.robot.mock else 'grasp_attempt_unverified'
        self.move([self.grasp[2],self.grasp[1]],lambda:self.issue(
            lambda cb:self.robot.grip(False,cb),lambda:self.move([self.grasp[0]],self.return_home)))

    def finish_tick(self,now):
        if self.robot.busy or self.ball_sequence.busy:
            if now>self.finish_deadline:self.pause('回正未开始：当前动作未能停止，请确认控制器状态后再点结束并回正')
            return
        if self.mode=='soft' and not self.released:
            self.phase='waiting_release';self.message='请松手，然后点击确认松手';return
        if not self.robot.stationary():
            self.release_since=None
            if now>self.finish_deadline:self.pause('回正未开始：关节持续运动，未发送回正动作')
            return
        if self.release_since is None:self.release_since=now
        if now-self.release_since<.7:return
        self.release_since=None
        if self.phase=='waiting_switch':self.finish_switch()
        else:self.finish_open()

    def finish_open(self):
        # Open first so a held ball is set down where it was, not dropped at the front pose.
        if not (self.robot.mock or self.robot.gripper_enabled):self.finish_switch();return
        generation=self.generation
        def opened(ok,message):
            if generation!=self.generation:return
            if not ok:self.get_logger().warning('gripper not confirmed open before homing: '+message)
            self.finish_switch()
        self.set_state('transition','finish_opening','回正前张开夹爪')
        ok,reason=self.robot.grip(False,opened)
        if not ok:opened(False,reason)

    def finish_switch(self):
        generation=self.generation
        def switched(ok,message):
            if generation!=self.generation:return
            if ok:self.wait_settled(self.return_home);return
            # A refused preflight switches nothing; retry once the controllers settle.
            if not self.robot.modes.busy and time.monotonic()<self.finish_deadline:
                self.set_state('finishing','waiting_switch','等待控制器静止后回正：'+message);return
            self.pause('回正未完成：'+message)
        self.set_state('transition','finish_switching','切换阻抗保持，准备回正')
        self.robot.hold()
        ok,reason=self.robot.mode('impedance',switched)
        if not ok:switched(False,reason)

    def return_home(self):
        table_return=self.mode=='ball' and self.ball_follow_only
        self.set_state('transition','returning','回到桌面观察姿态' if table_return else '缓慢回到朝人待机姿态')
        self.move_to_pose([self.table if table_return else self.front],self.idle)

    def idle(self):
        table_return=self.mode=='ball' and self.ball_follow_only
        self.mode=''
        self.sample=None
        self.set_state('idle','waiting','桌面视角就绪；手势 2 跟踪小球' if table_return else '朝人静止待机；举手势 1/2/3/4')

    def pause(self,message):
        self.ball_sequence.stop()
        self.generation+=1
        self.pending=None
        self.set_state('paused','stopped',message)
        self.robot.stop()

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.last_tick);self.last_tick=now
        self.robot.tick()
        self.ball_sequence.tick()
        if self.robot.fault and self.state!='paused':self.pause(self.robot.fault)
        if self.state=='finishing':self.finish_tick(now)
        if self.pending:
            if now>self.pending_deadline:self.pause('controller did not settle; no new motion sent')
            elif self.robot.mock or self.robot.motion.ready()=='':
                callback,self.pending=self.pending,None
                callback()
        if self.state=='active' and self.mode in ('face','ball'):
            if self.phase=='dock_check':self.dock_tick(now)
            elif self.robot.servo:self.tracking_tick(now,dt)

    def prepare_shutdown(self):self.pause('shutdown')

    def destroy_node(self):
        self.ball_sequence.destroy()
        self.robot.destroy()
        return super().destroy_node()


def main():run(Game)
