"""Deterministic gameplay helpers; no ROS or hardware side effects."""
import math
from ghost_game_orchestrator.mock_trajectory import minimum_quintic_duration
from .motion import JOINTS, LOWER, UPPER


def pose(value):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError('pose must contain six joint positions')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
           or not lo <= x <= hi for x, lo, hi in zip(value, LOWER, UPPER)):
        raise ValueError('pose outside Rebot limits')
    return [float(x) for x in value]


def path_points(start, targets, speed=.12, minimum=1., acceleration=None):
    start = pose(start)
    if not targets or not 0 < speed <= .6 or not math.isfinite(minimum) or minimum < .2:
        raise ValueError('invalid path settings')
    if acceleration is not None and (not math.isfinite(acceleration) or acceleration<=0):
        raise ValueError('invalid acceleration limit')
    points, elapsed, previous = [(.2, start)], .2, start
    for target in targets:
        target = pose(target)
        duration=minimum_quintic_duration(previous, target, [speed]*6, minimum)
        if acceleration is not None:
            # Zero endpoint velocity/acceleration gives a quintic segment whose
            # peak acceleration is (10/sqrt(3))*distance/duration**2.
            distance=max(abs(a-b) for a,b in zip(previous,target))
            duration=max(duration,math.sqrt((10/math.sqrt(3))*distance/acceleration))
        elapsed += duration
        points.append((elapsed, target))
        previous = target
    if elapsed > 60.:
        raise ValueError('path longer than 60 seconds; split into separate stages')
    return points


class DigitGate:
    def __init__(self, dwell=.8, release=.3, gap=.25):
        self.dwell, self.release, self.gap = dwell, release, gap
        self.candidate, self.since, self.last = 0, 0., None
        self.armed, self.release_since = True, None

    def update(self, digit, now, enabled=True, valid=True):
        if not enabled:
            self.armed = False
        if not valid or (self.last is not None and now-self.last > self.gap):
            self.candidate, self.release_since = 0, None
        self.last = now
        if not valid:
            return None
        if digit == 0:
            self.candidate = 0
            if self.release_since is None:
                self.release_since = now
            if enabled and now-self.release_since >= self.release:
                self.armed = True
            return None
        self.release_since = None
        if digit not in (1,2,3,4) or not enabled or not self.armed:
            self.candidate = 0
            return None
        if digit != self.candidate:
            self.candidate, self.since = digit, now
        if now-self.since >= self.dwell:
            self.armed, self.candidate = False, 0
            return digit
        return None


def digit_from_landmarks(points):
    """1=index, 2=index+middle, 3=+ring, 4=all four; thumb folded.

    Uses joint angles and wrist distances rather than image vertical direction.
    Ambiguous/bent fingers and an extended thumb do not select a mode.
    """
    if len(points) != 21 or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        return 0
    def dist(a,b): return math.dist(points[a],points[b])
    def angle(a,b,c):
        u=[points[a][i]-points[b][i] for i in range(3)]
        v=[points[c][i]-points[b][i] for i in range(3)]
        norm=math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
        return math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(u,v))/norm)))) if norm>1e-9 else 0.
    span=dist(5,17)
    if span < .03:
        return 0
    if angle(2,3,4)>155 and dist(4,5)>span*.8:
        return 0
    flags=[]
    for mcp,pip,dip,tip in [(5,6,7,8),(9,10,11,12),(13,14,15,16),(17,18,19,20)]:
        extended=angle(mcp,pip,tip)>155 and angle(pip,dip,tip)>155 and dist(0,tip)>dist(0,pip)*1.12
        folded=dist(0,tip)<dist(0,pip)*1.05
        if not extended and not folded:
            return 0
        flags.append(extended)
    patterns={(True,False,False,False):1,(True,True,False,False):2,
              (True,True,True,False):3,(True,True,True,True):4}
    return patterns.get(tuple(flags),0)


class StillGate:
    """Require a stationary target AND a stationary camera; no false stops while panning."""
    def __init__(self, dwell=.8, pixel_tolerance=.025, joint_tolerance=.015):
        self.dwell, self.pixel_tolerance, self.joint_tolerance = dwell,pixel_tolerance,joint_tolerance
        self.reset()

    def reset(self):
        self.anchor, self.since, self.last = None, None, None

    def update(self, xy, joints, now):
        current=(list(xy),list(joints))
        if self.last is not None and now-self.last>.3:
            self.reset()
        self.last=now
        if self.anchor is None or max(abs(a-b) for a,b in zip(xy,self.anchor[0]))>self.pixel_tolerance or max(abs(a-b) for a,b in zip(joints,self.anchor[1]))>self.joint_tolerance:
            self.anchor,self.since=current,now
        return now-self.since>=self.dwell
