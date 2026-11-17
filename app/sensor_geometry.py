"""Geometric edge-tracking poses; forward-only nominal line tracking."""
import math
from dataclasses import dataclass
from actions import Turn, LineTrack


def get_line_normal(a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
    if length<=1e-9: raise ValueError('Degenerate tracking segment')
    return (-dy/length,dx/length)


def get_line_edge_point(point,a,b,side,line_width_mm,pixels_per_mm):
    if side not in ('left','right') or min(line_width_mm,pixels_per_mm)<=0: raise ValueError('Invalid tracking geometry')
    nx,ny=get_line_normal(a,b); offset=(1 if side=='right' else -1)*line_width_mm*pixels_per_mm/2
    return (point[0]+offset*nx,point[1]+offset*ny)


def sensor_to_robot_pose(sensor, heading, forward_mm, right_mm, pixels_per_mm):
    from planner import Pose, normalize_angle
    if pixels_per_mm<=0: raise ValueError('Invalid scale')
    a=math.radians(heading); c,s=math.cos(a),math.sin(a)
    return Pose(sensor[0]-pixels_per_mm*(forward_mm*c-right_mm*s),sensor[1]-pixels_per_mm*(forward_mm*s+right_mm*c),normalize_angle(heading))


@dataclass(frozen=True)
class TrackingLeg:
    start: object
    end: object
    distance_mm: float
    side: str


def generate_tracking_poses(a,b,robot,pixels_per_mm,line_width_mm=20,inset_mm=15):
    """Inset each endpoint to avoid ambiguous black junction regions.
    Width is a user estimate, not inferred from the raster. Side names are geometric.
    """
    if min(pixels_per_mm,line_width_mm)<=0 or inset_mm<0: raise ValueError('Invalid geometry')
    dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
    inset=max(inset_mm,line_width_mm/2+getattr(robot,'sensor_diameter_mm',0)/2)*pixels_per_mm
    if length<=2*inset+1e-6: return []
    ux,uy=dx/length,dy/length; heading=math.degrees(math.atan2(dy,dx))
    p=(a[0]+inset*ux,a[1]+inset*uy); q=(b[0]-inset*ux,b[1]-inset*uy)
    result=[]
    for side in ('left','right'):
        s=get_line_edge_point(p,a,b,side,line_width_mm,pixels_per_mm)
        t=get_line_edge_point(q,a,b,side,line_width_mm,pixels_per_mm)
        args=(heading,robot.sensor_x_mm,robot.sensor_y_mm,pixels_per_mm)
        result.append(TrackingLeg(sensor_to_robot_pose(s,*args),sensor_to_robot_pose(t,*args),(length-2*inset)/pixels_per_mm,side))
    return result


def generate_line_acquisition(start,target,pixels_per_mm):
    """Ideal dead-reckoning approach with required final heading; no feedback."""
    from planner import generate_direct_candidates,calculate_end_pose,normalize_angle
    result=[]
    for actions in generate_direct_candidates(start,(target.x,target.y),pixels_per_mm):
        end=calculate_end_pose(start,actions,pixels_per_mm)
        angle=normalize_angle(target.heading-end.heading)
        result.append(actions+((Turn(angle),) if abs(angle)>1e-6 else ()))
    return result


def raster_edge_validator(rgb, pixels_per_mm, robot, min_match=0.90, max_gap_mm=10.0):
    """Return a nominal edge validator using the same dark-neutral threshold as extraction.
    Checks a pair of probes on opposite sides of the sensor path, not PID physics.
    """
    if not 0 < min_match <= 1 or max_gap_mm < 0:
        raise ValueError('Invalid raster validation tolerance')
    import numpy as np
    rgb=np.asarray(rgb)[:,:,:3].astype(np.int16)
    hi=rgb.max(axis=2); mask=(hi<90)&((hi-rgb.min(axis=2))<45)
    h,w=mask.shape
    def dark(x,y):
        ix,iy=round(x),round(y)
        return 0<=ix<w and 0<=iy<h and bool(mask[iy,ix])
    def valid(leg):
        a=math.radians(leg.start.heading); ux,uy=math.cos(a),math.sin(a); nx,ny=-uy,ux
        sx=leg.start.x+pixels_per_mm*(robot.sensor_x_mm*ux+robot.sensor_y_mm*nx)
        sy=leg.start.y+pixels_per_mm*(robot.sensor_x_mm*uy+robot.sensor_y_mm*ny)
        probe=max(1.5,min(5.0,getattr(robot,'sensor_diameter_mm',18)/3))*pixels_per_mm
        sign=1 if leg.side=='right' else -1
        n=max(1,math.ceil(leg.distance_mm/5))
        matches=[]
        for i in range(n+1):
            x=sx+leg.distance_mm*pixels_per_mm*i/n*ux; y=sy+leg.distance_mm*pixels_per_mm*i/n*uy
            matches.append(dark(x-sign*probe*nx,y-sign*probe*ny) and not dark(x+sign*probe*nx,y+sign*probe*ny))
        if not matches[0] or not matches[-1] or sum(matches)/len(matches)<min_match:
            return False
        consecutive=0
        for match in matches:
            consecutive=0 if match else consecutive+1
            if consecutive*leg.distance_mm/n>max_gap_mm+1e-9:
                return False
        return True
    return valid
