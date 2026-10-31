import json
import time
from concurrent.futures import Future
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from armie.senses import Senses
from armie.game import Game
from test_ros import graph


def test_latest_only_ordered_images_and_inflight_context_rejection():
    with graph([Senses],['senses_backend:=camera']) as (nodes,exe):
        senses=nodes[0];senses.timer.cancel()
        msg=CompressedImage(data=b'image')
        msg.header.stamp=senses.get_clock().now().to_msg()
        senses.on_image(msg);first=senses.latest
        senses.on_image(msg);assert senses.latest==first
        msg.header.stamp.nanosec=max(0,msg.header.stamp.nanosec-1)
        senses.on_image(msg);assert senses.latest==first
        pending=Future();pending.set_result((dict(valid=True,digit=3,ball=None),None))
        senses.future=pending;senses.inflight=(first[1],0,senses.context_generation)
        senses.on_state(String(data=json.dumps(dict(schema_version=2,state='paused',mode='ball',phase='preview',round_id=0))))
        assert senses.latest is None
        senses.last_at=time.monotonic()
        senses.tick();assert senses.sequence==0


def test_ball_uses_capture_age_and_invalid_observation_brakes_without_predicting():
    with graph([Game],['ball_prediction_enabled:=true']) as (nodes,exe):
        game=nodes[0];game.timer.cancel();game.state='active';game.mode='ball'
        stamp=game.get_clock().now().nanoseconds*1e-9
        value=dict(schema_version=1,source='external',stamp=stamp-.08,session='test',sequence=1,
            round_id=game.round_id,valid=True,digit=0,ball=dict(x=.2,y=0.,width=.1,height=.1,score=.9))
        game.observation(String(data=json.dumps(value)))
        assert game.sample is not None
        assert time.monotonic()-game.sample.received_at>=.08
        value.update(sequence=2,stamp=stamp,ball=None)
        game.observation(String(data=json.dumps(value)))
        assert game.sample is None and game.ball_prediction.point is None
        value.update(sequence=3,stamp=stamp-.13,ball=dict(x=.2,y=0.,width=.1,height=.1,score=.9))
        game.observation(String(data=json.dumps(value)))
        assert game.sample is None
