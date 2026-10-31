"""RGB-D ball localization and non-executable curiosity/grasp waypoints."""
import math
import numpy as np


def rotation(q):
    x,y,z,w=np.asarray(q,dtype=float);norm=x*x+y*y+z*z+w*w
    if not math.isfinite(norm) or abs(norm-1)>1e-3:raise ValueError('invalid camera quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def locate_ball(depth,k,ball,translation,quaternion):
    """Fit a sphere to the central visible surface; reject weak/non-spherical fits."""
    h,w=depth.shape;fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    if min(fx,fy)<=0:raise ValueError('invalid camera intrinsics')
    u=(ball['x']+1)*w/2;v=(ball['y']+1)*h/2
    rx=ball['width']*w/4*.65;ry=ball['height']*h/4*.65
    if min(rx,ry)<5:raise ValueError('target too small for depth fit')
    ys,xs=np.mgrid[max(0,int(v-ry)):min(h,int(v+ry)+1):2,max(0,int(u-rx)):min(w,int(u+rx)+1):2]
    z=depth[ys,xs];valid=(((xs-u)/rx)**2+((ys-v)/ry)**2<=1)&np.isfinite(z)&(z>.08)&(z<1.2)
    if valid.sum()<40:raise ValueError('insufficient valid ball depth')
    z=z[valid];x=xs[valid];y=ys[valid]
    near=np.abs(z-np.median(z))<.04
    z=z[near];x=x[near];y=y[near]
    p=np.column_stack(((x-cx)*z/fx,(y-cy)*z/fy,z))
    if len(p)<40:raise ValueError('inconsistent depth')
    origin=np.mean(p,axis=0);local=p-origin
    solution,_,rank,_=np.linalg.lstsq(np.column_stack((2*local,np.ones(len(p)))),np.sum(local*local,axis=1),rcond=None)
    if rank<4:raise ValueError('flat target cannot define a ball')
    center=solution[:3]+origin;r2=solution[3]+np.dot(solution[:3],solution[:3])
    if r2<=0:raise ValueError('invalid ball fit')
    radius=math.sqrt(r2);residual=np.abs(np.linalg.norm(p-center,axis=1)-radius)
    if not .015<=radius<=.085 or np.quantile(residual,.9)>.006:
        raise ValueError('sphere fit does not reliably describe selected ball')
    if center[2]<np.median(z) or center[2]-np.median(z)>radius:
        raise ValueError('ball center is not behind visible surface')
    world=rotation(quaternion)@center+np.asarray(translation)
    return dict(center=world.tolist(),radius=radius,fit_p90_m=float(np.quantile(residual,.9)),points=len(p))


class WorldStillGate:
    def __init__(self,dwell=3.,tolerance=.01):
        self.dwell=dwell;self.tolerance=tolerance;self.reset()
    def reset(self):self.anchor=None;self.since=None;self.last=None
    def update(self,center,stamp):
        p=np.asarray(center,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all() or not math.isfinite(stamp):
            self.reset();return False
        if self.last is not None and stamp<=self.last:return False
        if self.last is not None and stamp-self.last>.2:self.reset()
        if self.anchor is None or np.linalg.norm(p-self.anchor)>self.tolerance:
            self.anchor=p;self.since=stamp
        self.last=stamp
        return stamp-self.since>=self.dwell


def preview_waypoints(center,radius,yaw=0.,orientation=None):
    """Preserve a supplied tool attitude; distances remain preview assumptions."""
    x,y,z=center
    if not all(math.isfinite(v) for v in (*center,radius,yaw)):raise ValueError('non-finite geometry')
    if not .015<=radius<=.085:raise ValueError('invalid ball radius')
    q=list(orientation) if orientation is not None else [math.cos(yaw/2),math.sin(yaw/2),0.,0.]
    rot=rotation(q);back=-rot[:,2]
    side=np.array([rot[0,0],rot[1,0],0.]);side/=max(np.linalg.norm(side),1e-9)
    # At the side views, pitch slightly inward to give a curious inspection.
    result=[]
    for name,dx,dz,tilt in [('approach',0,.10,0),('look_left',-.025,.10,-.12),
                           ('look_right',.025,.10,.12),('pregrasp',0,.07,0),
                           ('grasp',0,0,0),('lift',0,.12,0)]:
        # q * rotation about the local Y axis.
        a,b,c,d=q;s=math.sin(tilt/2);co=math.cos(tilt/2)
        orientation=[a*co-c*s,b*co+d*s,c*co+a*s,d*co-b*s]
        result.append(dict(stage=name,position=(np.asarray(center)+(np.array([0.,0.,dz]) if name=='lift' else back*dz+side*dx)).tolist(),orientation=orientation,
                           gripper='close' if name=='grasp' else ('hold' if name=='lift' else 'open')))
    return result


def wrapup_waypoints(waypoints,home,success):
    """Success: set the ball down 1 cm above where it was taken, back off along the
    approach axis, then return to the sequence start. Failure: straight back to start."""
    stages={w['stage']:w for w in waypoints}
    back=dict(home,stage='home')
    if not success:return [back]
    place=dict(stages['grasp'],stage='place',position=(np.asarray(stages['grasp']['position'])+[0.,0.,.01]).tolist(),gripper='open')
    clear=dict(stages['pregrasp'],stage='clear')
    return [place,clear,back]


def grasp_evidence(finger,frames,on_table,empty=.92,minimum=.1,min_frames=3):
    """Both sources must agree: fingers stopped short of fully closed AND a live
    camera no longer localizes the ball at its table position."""
    if finger is None or not math.isfinite(finger):return False,'夹爪反馈缺失'
    if finger>=empty:return False,'夹爪完全闭合，未夹到物体'
    if finger<minimum:return False,'夹爪未闭合'
    if frames<min_frames:return False,'相机观测不足，无法确认'
    if on_table:return False,'小球仍在桌面原位'
    return True,'夹爪停在物体上且小球已离开桌面'
