import math
import numpy as np
import pytest
from armie.seaweed import seaweed_path

HOME=[-.157,.2,.2,0.,0.,0.]

@pytest.mark.parametrize('amplitude,speed',[(1.,1.),(1.,.5),(.25,.5),(1.,1.6),(.25,1.6)])
def test_wave_interpolant_limits_endpoints_and_continuity(amplitude,speed):
    points=seaweed_path(HOME,HOME,amplitude,speed)
    assert points[0][1]==points[-1][1]==HOME
    assert points[-1][2:]==([0.]*6,[0.]*6)
    assert points[-1][0]<60
    assert sum(any(abs(v)>.001 for v in row[2]) for row in points)>len(points)*.75
    ranges=np.ptp(np.array([row[1] for row in points]),axis=0)
    assert ranges[1]>.35*amplitude and ranges[2]>.45*amplitude
    assert all(ranges>.25*amplitude)
    # Check the actual quintic interpolant used by JTC, including extrema
    # between samples, rather than checking only the supplied knot values.
    matrix=np.array([[1,1,1],[3,4,5],[6,12,20]],dtype=float)
    for first,second in zip(points,points[1:]):
        dt=second[0]-first[0]
        assert dt>0
        for j in range(6):
            c=np.zeros(6)
            c[:3]=[first[1][j],first[2][j]*dt,first[3][j]*dt*dt/2]
            rhs=[second[1][j]-sum(c),second[2][j]*dt-c[1]-2*c[2],second[3][j]*dt*dt-2*c[2]]
            c[3:]=np.linalg.solve(matrix,rhs)
            for derivative in range(3):
                d=np.polynomial.polynomial.polyder(c,derivative)
                roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(d))
                candidates=[0.,1.]+[r.real for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
                values=np.polynomial.polynomial.polyval(candidates,d)/dt**derivative
                if derivative==0 and j in (1,2):assert min(values)>=.15-1e-9
                if derivative==1:
                    assert max(abs(values))<=.4*speed+1e-7
                    assert max(abs(values))<=([.65,.6,.6,.65,.65,.65][j])+1e-7
                if derivative==2:assert max(abs(values))<=.65*speed**2+1e-7

def test_wave_rejects_unsafe_anchor_and_invalid_controls():
    for amp,rate in [(1.1,1.),(1.,.4),(math.nan,1.),(1.,math.inf),(1.,1.61)]:
        with pytest.raises(ValueError):seaweed_path(HOME,HOME,amp,rate)
    with pytest.raises(ValueError):seaweed_path([0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.])
    with pytest.raises(ValueError):seaweed_path([2.75,.2,.2,0.,0.,0.],[2.75,.2,.2,0.,0.,0.])


def test_faster_wave_preserves_amplitude_and_retimes_derivatives():
    old=seaweed_path(HOME,HOME,1.,1.)
    fast=seaweed_path(HOME,HOME,1.,1.6)
    approach=old[1][0]
    assert fast[-1][0]==pytest.approx(approach+(old[-1][0]-approach)/1.6)
    assert np.ptp(np.array([p[1] for p in fast]),axis=0)==pytest.approx(
        np.ptp(np.array([p[1] for p in old]),axis=0),abs=.002)
