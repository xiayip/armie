import json
import math
import time
import pytest
from std_msgs.msg import String
from armie.game import Game
from armie.senses import Senses,detect_ball
from armie.game_core import DigitGate,StillGate,path_points,digit_from_landmarks
from test_ros import graph,until


def advance(executor,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:executor.spin_once(timeout_sec=.01)


def test_digit_dwell_release_and_no_retrigger_during_active_mode():
    gate=DigitGate()
    events=[]
    for i in range(21):
        event=gate.update(2,i*.1)
        if event:events.append(event)
    assert events==[2]
    for i in range(21,30):assert gate.update(3,i*.1,enabled=False) is None
    for i in range(30,40):assert gate.update(3,i*.1) is None
    for i in range(40,45):gate.update(0,i*.1)
    results=[gate.update(3,i*.1) for i in range(45,56)]
    assert results.count(3)==1


def test_missing_frames_cannot_complete_digit_dwell():
    gate=DigitGate()
    assert gate.update(1,0.) is None
    assert gate.update(1,1.) is None
    assert gate.update(1,1.1,valid=False) is None
    assert gate.update(1,1.2) is None


def test_moving_camera_does_not_make_ball_stationary():
    gate=StillGate()
    for i in range(20):
        assert not gate.update((0.,0.),[i*.02]*6,i*.1)
    gate.reset()
    assert any(gate.update((0.,0.),[0.]*6,i*.1) for i in range(12))


def test_position_paths_have_bounded_quintic_peak_speed():
    start=[0.,.6,.8,0.,0.,0.]
    points=path_points(start,[[0.,.8,1.,.2,0.,0.],start],.2,1.4)
    assert points[-1][1]==start
    for (t0,q0),(t1,q1) in zip(points,points[1:]):
        assert max(1.875*abs(a-b)/(t1-t0) for a,b in zip(q0,q1))<=.2+1e-9
    with pytest.raises(ValueError):path_points(start,[[0.,.6,.8,1.6,0.,0.]])


def test_ball_detection_rejects_ambiguous_objects_and_rectangles():
    import cv2
    import numpy as np
    frame=np.zeros((240,320,3),dtype=np.uint8)
    cv2.circle(frame,(160,120),20,(0,140,255),-1)
    result=detect_ball(frame,[5,100,80],[30,255,255])
    assert result is not None and abs(result['x'])<.02
    cv2.circle(frame,(60,120),20,(0,140,255),-1)
    assert detect_ball(frame,[5,100,80],[30,255,255]) is None
    frame[:]=0
    cv2.rectangle(frame,(30,50),(150,70),(0,140,255),-1)
    assert detect_ball(frame,[5,100,80],[30,255,255]) is None


def test_four_mock_modes_return_to_idle_and_soft_requires_release():
    with graph([Game,Senses],['mock_step_sec:=0.03','face_duration_sec:=1.0']) as (nodes,exe):
        game,senses,probe=nodes
        assert game.state=='idle'
        assert game.command('1')[0]
        until(exe,lambda:game.phase=='dragging')
        assert game.robot.modes.mode=='damping'
        assert not game.command('3')[0]
        assert game.command('finish')[0]
        advance(exe,.3)
        assert game.phase=='waiting_release'
        assert game.command('release')[0]
        until(exe,lambda:game.state=='idle')
        assert game.robot.modes.mode=='impedance'
        assert game.command('3')[0]
        until(exe,lambda:game.state=='idle')
        assert game.command('4')[0]
        until(exe,lambda:game.robot.servo)
        until(exe,lambda:game.state=='idle',timeout=5.)
        assert game.command('2')[0]
        until(exe,lambda:game.state=='idle',timeout=10.)
        assert game.outcome=='curious_attempt_only'
        assert game.robot.current==game.front


def test_stop_latches_without_home_and_drops_stale_round_observations():
    with graph([Game],['mock_step_sec:=0.05']) as (nodes,exe):
        game,probe=nodes
        assert game.command('4')[0]
        until(exe,lambda:game.robot.servo)
        game.command('stop')
        assert game.state=='paused' and not game.robot.servo
        original=list(game.robot.current)
        advance(exe,.3)
        assert game.robot.current==original
        assert not game.command('2')[0]
        game.command('ready')
        until(exe,lambda:game.state=='idle')
        msg=dict(schema_version=1,source='external',stamp=game.get_clock().now().nanoseconds*1e-9,
                 session='test',sequence=1,round_id=game.round_id-1,valid=True,digit=1)
        game.observation(String(data=json.dumps(msg)))
        assert game.gate.candidate==0


def test_real_motion_is_opt_in_and_real_game_rejects_mock_observations():
    with graph([Game],['motion_backend:=trajectory','front_pose_json:=\'[0.0,0.6,0.8,0.0,0.0,0.0]\'']) as (nodes,exe):
        game=nodes[0]
        assert game.state=='paused'
        game.state='idle'
        assert not game.command('1')[0]
        for i in range(15):
            msg=dict(schema_version=1,source='mock',stamp=game.get_clock().now().nanoseconds*1e-9,
                     session='fake',sequence=i,round_id=0,valid=True,digit=1)
            game.observation(String(data=json.dumps(msg)))
        assert game.gate.candidate==0 and not game.robot.busy


def test_mock_grasp_reports_simulation_not_success_evidence():
    path=[[0.,.6,.8,.2,0.,0.],[0.,.65,.8,.2,0.,0.],[0.,.6,.8,.2,0.,0.]]
    with graph([Game,Senses],['mock_step_sec:=0.02','grasp_enabled:=true',
                             "grasp_path_json:='"+json.dumps(path)+"'"]) as (nodes,exe):
        game=nodes[0]
        assert game.command('2')[0]
        until(exe,lambda:game.state=='idle',timeout=12.)
        assert game.outcome=='simulated_grasp'

@pytest.mark.parametrize('digit',[1,2,3,4])
def test_digit_geometry_for_numbered_fingers_is_rotation_invariant(digit):
    points=[[.5,.9,0.] for _ in range(21)]
    points[1:5]=[[.35,.85,0.],[.3,.75,0.],[.35,.65,0.],[.45,.65,0.]]
    for i,(mcp,pip,dip,tip) in enumerate([(5,6,7,8),(9,10,11,12),(13,14,15,16),(17,18,19,20)]):
        x=.4+i*.1
        points[mcp]=[x,.65,0.]
        points[pip]=[x,.45,0.]
        points[dip]=[x,.3 if i<digit else .55,0.]
        points[tip]=[x,.2 if i<digit else .7,0.]
    assert digit_from_landmarks(points)==digit
    rotated=[[-y,x,z] for x,y,z in points]
    assert digit_from_landmarks(rotated)==digit


def test_camera_compressed_image_to_ball_observation_and_stale_frame_rejection():
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    from rclpy.qos import qos_profile_sensor_data
    with graph([Senses],['senses_backend:=camera']) as (nodes,exe):
        senses,probe=nodes
        senses.state,senses.mode,senses.round_id='active','ball',7
        received=[]
        sub=probe.create_subscription(String,'/armie/observation',lambda m:received.append(json.loads(m.data)),10)
        pub=probe.create_publisher(CompressedImage,'/camera/color/image_raw/compressed',qos_profile_sensor_data)
        until(exe,lambda:pub.get_subscription_count()>0)
        image=np.zeros((240,320,3),dtype=np.uint8)
        cv2.circle(image,(160,120),22,(0,140,255),-1)
        _,jpeg=cv2.imencode('.jpg',image)
        msg=CompressedImage(format='jpeg',data=jpeg.tobytes())
        msg.header.stamp=probe.get_clock().now().to_msg()
        pub.publish(msg)
        until(exe,lambda:any(v.get('ball') for v in received))
        result=next(v for v in received if v.get('ball'))
        assert result['source']=='camera' and result['round_id']==7
        assert abs(result['ball']['x'])<.02
        before=senses.sequence
        msg.header.stamp.sec-=10
        pub.publish(msg)
        advance(exe,.2)
        assert senses.sequence==before


def test_ros_digit_observations_trigger_one_menu_selection():
    with graph([Game],['mock_step_sec:=0.02']) as (nodes,exe):
        game,probe=nodes
        pub=probe.create_publisher(String,'/armie/observation',10)
        until(exe,lambda:pub.get_subscription_count()>0)
        for i in range(15):
            value=dict(schema_version=1,stamp=probe.get_clock().now().nanoseconds*1e-9,source='external',
                       session='digits',sequence=i,round_id=game.round_id,valid=True,digit=1)
            pub.publish(String(data=json.dumps(value)))
            advance(exe,.08)
        assert game.mode=='soft' and game.phase=='dragging' and game.round_id==1


def test_rejected_topic_command_reports_reason_without_starting_motion():
    from armie.common import STATE_QOS
    with graph([Game]) as (nodes,exe):
        game,probe=nodes
        states=[]
        sub=probe.create_subscription(String,'/armie/state',lambda m:states.append(json.loads(m.data)),STATE_QOS)
        pub=probe.create_publisher(String,'/armie/command',10)
        until(exe,lambda:pub.get_subscription_count()>0)
        pub.publish(String(data='999'))
        until(exe,lambda:any(s['message']=='unknown command' for s in states))
        assert game.state=='idle' and not game.robot.busy


def test_follow_only_finish_returns_to_table_without_grasp_or_front_detour():
    with graph([Game,Senses],['mock_step_sec:=0.03','ball_follow_only:=true']) as (nodes,exe):
        game,senses,probe=nodes
        assert game.command('2')[0]
        until(exe,lambda:game.phase=='watching')
        assert game.command('finish')[0]
        until(exe,lambda:game.state=='idle')
        assert game.robot.current==game.table
        assert '桌面' in game.message
