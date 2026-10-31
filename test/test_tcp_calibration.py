import numpy as np
import pytest
from armie.tcp_calibration import solve_pivot
from armie.ball_grasp_geometry import rotation


def samples(noise=0.):
    rng=np.random.default_rng(21);offset=np.array([.003,-.002,.147]);fixed=np.array([.25,-.1,.07]);rows=[]
    for axis,angle in [([1,0,0],-.4),([1,0,0],.4),([0,1,0],-.4),([0,1,0],.4),([0,0,1],-.3),([0,0,1],.3)]:
        q=[*(np.array(axis)*np.sin(angle/2)),np.cos(angle/2)]
        t=fixed-rotation(q)@offset+rng.normal(0,noise,3)
        rows.append(dict(rotation=q,translation=t.tolist()))
    return rows


def test_recovers_offset_and_fixed_point_without_applying():
    r=solve_pivot(samples(.0002))
    assert r['offset_xyz_m']==pytest.approx([.003,-.002,.147],abs=.001)
    assert r['fixed_point_base_m']==pytest.approx([.25,-.1,.07],abs=.001)
    assert r['applied'] is False and r['orientation_calibrated'] is False


def test_rejects_duplicate_or_inconsistent_alignment_and_insufficient_samples():
    with pytest.raises(ValueError,match='six'):solve_pivot(samples()[:3])
    with pytest.raises(ValueError,match='diversity'):solve_pivot([samples()[0]]*6)
    x=samples();x[-1]['translation'][0]+=.05
    with pytest.raises(ValueError,match='residual'):solve_pivot(x)
