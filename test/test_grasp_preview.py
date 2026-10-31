import math
import numpy as np
import pytest
from armie.ball_grasp_geometry import WorldStillGate,locate_ball,preview_waypoints


def test_three_seconds_require_continuous_world_stationarity_and_fresh_samples():
    g=WorldStillGate()
    for i in range(30):assert not g.update([.2,0,.1],i*.1)
    assert g.update([.2,0,.1],3.)
    assert not g.update([.2,0,.1],3.)
    assert not g.update([.23,0,.1],3.1)
    assert not g.update([.23,0,.1],4.)
    g.reset()
    for i in range(50):assert not g.update([.2+i*.003,0,.1],i*.1)


def test_rgbd_sphere_center_and_invalid_depth():
    h,w=160,200;f=200.;cx=w/2;cy=h/2
    y,x=np.mgrid[:h,:w];rays=np.stack(((x-cx)/f,(y-cy)/f,np.ones((h,w))),axis=-1)
    center=np.array([0.,0.,.4]);radius=.04
    a=np.sum(rays*rays,axis=-1);b=-2*np.sum(rays*center,axis=-1);c=center@center-radius**2
    disc=b*b-4*a*c;depth=np.where(disc>=0,(-b-np.sqrt(np.maximum(0,disc)))/(2*a),0)
    ball=dict(x=0.,y=0.,width=.4,height=.5)
    result=locate_ball(depth,[f,0,cx,0,f,cy,0,0,1],ball,[.1,.2,.3],[0,0,0,1])
    assert result['center']==pytest.approx([.1,.2,.7],abs=1e-5)
    assert result['radius']==pytest.approx(.04,abs=1e-5)
    with pytest.raises(ValueError):locate_ball(np.zeros_like(depth),[f,0,cx,0,f,cy,0,0,1],ball,[0,0,0],[0,0,0,1])
    with pytest.raises(ValueError):locate_ball(np.full_like(depth,.4),[f,0,cx,0,f,cy,0,0,1],ball,[0,0,0],[0,0,0,1])


def test_preview_sequence_closes_only_at_grasp_and_lifts_without_release():
    p=preview_waypoints([.2,.1,.04],.04)
    assert [x['stage'] for x in p]==['approach','look_left','look_right','pregrasp','grasp','lift']
    assert p[1]['position'][0]<.2<p[2]['position'][0]
    assert all(x['gripper']=='open' for x in p[:4])
    assert p[-2]['gripper']=='close' and p[-1]['gripper']=='hold'
    assert p[-1]['position'][2]>p[-2]['position'][2]
    for x in p:assert sum(q*q for q in x['orientation'])==pytest.approx(1.)


def test_ball_loss_waits_and_no_duration_timeout_in_continuous_mode():
    import time
    from armie.game import Game
    from test_ros import graph
    with graph([Game],['ball_wait_for_target:=true','ball_duration_sec:=0.0','ball_still_sec:=3.0']) as (nodes,exe):
        g=nodes[0];g.timer.cancel();g.mode='ball';g.state='active';g.robot.servo=True
        g.robot.reference=g.robot.current;g.robot.tracking_anchor=g.robot.current;g.sample=None
        now=time.monotonic();g.started=now-1000;g.last_seen=now-1000
        g.tracking_tick(now,.025)
        assert g.state=='active' and g.phase=='waiting_target'
        assert g.still.dwell==3.


def test_preview_node_starts_without_motion_interfaces(tmp_path):
    from armie.grasp_preview import GraspPreview
    from test_ros import graph
    with graph([GraspPreview],[f'output:={tmp_path}/preview.json']) as (nodes,exe):
        node=nodes[0]
        assert node.client.srv_name=='/compute_cartesian_path'
        node.invalidate('missing depth')
        import json
        result=json.loads((tmp_path/'preview.json').read_text())
        assert result['execution_allowed'] is False and result['status']=='waiting'


def test_new_target_after_completed_numpy_plan_resets_without_truth_value_error(tmp_path,monkeypatch):
    import json
    from armie.grasp_preview import GraspPreview
    from sensor_msgs.msg import Image,CameraInfo,JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from test_ros import graph
    with graph([GraspPreview],[f'output:={tmp_path}/preview.json']) as (nodes,exe):
        n=nodes[0];stamp=n.get_clock().now().to_msg();seconds=stamp.sec+stamp.nanosec*1e-9
        n.state={'state':'paused'};n.last_planned=np.array([.2,0,.04])
        n.info=CameraInfo(width=20,height=20);n.info.header.frame_id='camera'
        n.joints=JointState();n.joints.header.stamp=stamp
        im=Image(width=20,height=20,encoding='16UC1',step=40,data=np.full((20,20),200,dtype=np.uint16).tobytes());im.header.stamp=stamp;im.header.frame_id='camera';n.depth.append(im)
        tf=TransformStamped();tf.transform.rotation.w=1.
        monkeypatch.setattr(n.buffer,'lookup_transform',lambda *a:tf)
        monkeypatch.setattr('armie.grasp_preview.locate_ball',lambda *a:dict(center=[.24,0,.04],radius=.02))
        n.observe(String(data=json.dumps(dict(stamp=seconds,source='camera',valid=True,ball={'x':0.}))))
        result=json.loads((tmp_path/'preview.json').read_text())
        assert result['status']=='measuring_stillness' and n.last_planned is None
