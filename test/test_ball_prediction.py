import math
import pytest
from armie.ball_prediction import BallPrediction


def moving_prediction():
    p=BallPrediction()
    for i in range(12):p.update(10+i/30.,i/30.,0.,.9)
    return p


def test_prediction_compensates_capture_delay_with_bounded_lead():
    p=moving_prediction();now=p.stamp+.05
    actual=(now+.04)-10
    predicted=p.predict(now)
    assert abs(predicted[0]-actual)<.003
    assert abs(predicted[0]-actual)<abs(p.point[0]-actual)/10
    assert math.hypot(*[a-b for a,b in zip(predicted,p.point)])<=.10+1e-9


def test_reversal_jump_and_low_confidence_cancel_prediction():
    for dx,score in [(-.03,.9),(.3,.9),(.03,.72)]:
        p=moving_prediction();p.update(p.stamp+1/30,p.point[0]+dx,0.,score)
        assert p.predict(p.stamp+.05)==tuple(p.point)


def test_ordered_capture_time_loss_and_stationary_frames():
    p=moving_prediction();stamp=p.stamp;point=tuple(p.point)
    assert not p.update(stamp-.01,-.5,0.,.9)
    assert tuple(p.point)==point
    assert p.predict(stamp+.121)==point
    p.reset();assert p.predict(stamp) is None
    for i in range(10):p.update(20+i/30,.1,-.1,.9)
    assert p.predict(p.stamp+.05)==(.1,-.1)
    p.update(p.stamp+.2,.2,0.,.9)
    assert p.predict(p.stamp+.05)==(.2,0.)
    assert not p.update(float('nan'),0.,0.,.9)
    assert p.predict(30) is None


@pytest.mark.parametrize('args',[dict(lead=.1),dict(max_horizon=.2),dict(max_offset=.2)])
def test_invalid_prediction_bounds(args):
    with pytest.raises(ValueError):BallPrediction(**args)
