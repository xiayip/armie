"""Continuous six-axis choreography with zero-velocity/acceleration endpoints."""
import math

from .game_core import pose, path_points


def seaweed_path(start, anchor, amplitude=1., speed=1.):
    start, anchor = pose(start), pose(anchor)
    if not math.isfinite(amplitude) or not .25 <= amplitude <= 1.:
        raise ValueError('dance amplitude must be within [0.25, 1]')
    if not math.isfinite(speed) or not .5 <= speed <= 1.6:
        raise ValueError('dance speed must be within [0.5, 1.6]')
    # J2/J3 only lift from the commissioned home; never wave below it.
    bias = [0., .23, .30, 0., 0., 0.]
    size = [.25, .22, .30, .30, .30, .35]
    phase = [-1.4, 0., -.9, -1.8, -2.5, -3.1]
    omega = 6*math.pi  # Three waves, with a phase delay along the arm.

    def sample(s):
        x = math.pi*s
        sn, cs = math.sin(x), math.cos(x)
        envelope = sn**4
        de = 4*math.pi*sn**3*cs
        dde = 4*math.pi**2*(3*sn**2*cs**2-sn**4)
        q, v, a = [], [], []
        for home, offset, scale, shift in zip(anchor, bias, size, phase):
            w = omega*s+shift
            g = offset+scale*math.sin(w)
            dg = scale*omega*math.cos(w)
            ddg = -scale*omega**2*math.sin(w)
            q.append(home+amplitude*envelope*g)
            v.append(amplitude*(de*g+envelope*dg))
            a.append(amplitude*(dde*g+2*de*dg+envelope*ddg))
        return q, v, a

    # Dense derivative scan plus margin retimes the entire wave, preserving phase.
    samples = [sample(i/2000) for i in range(2001)]
    peak_v = max(abs(x) for _, v, _ in samples for x in v)
    peak_a = max(abs(x) for _, _, a in samples for x in a)
    duration = max(16., peak_v/.4, math.sqrt(peak_a/.65))*1.02/speed
    approach = path_points(start, [anchor], .12, 1.)
    offset = approach[-1][0]
    if duration+offset > 60.:
        raise ValueError('dance exceeds 60 seconds')
    points = [(t, q, [0.]*6, [0.]*6) for t,q in approach]
    steps = math.ceil(duration/.1)
    for i in range(1, steps+1):
        q, v, a = sample(i/steps)
        pose(q)
        # Keep the entire wave out of the controller's 0.15 rad soft-limit zone.
        if not .15 <= q[1] <= 2.99 or not .15 <= q[2] <= 2.99:
            raise ValueError('dance enters J2/J3 soft-limit buffer')
        points.append((offset+duration*i/steps, q,
                       [x/duration for x in v], [x/duration**2 for x in a]))
    points[-1] = (points[-1][0], list(anchor), [0.]*6, [0.]*6)
    return points
