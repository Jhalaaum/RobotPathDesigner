"""Nominal swept-footprint checks. Local geometry mm; world geometry pixels."""
import math
from dataclasses import dataclass
from shapely.geometry import Polygon, box
from shapely.prepared import prep
from actions import Drive, Turn, LineTrack


def transform_polygon(polygon, pose, pixels_per_mm):
    if pixels_per_mm <= 0: raise ValueError('Scale must be positive')
    a = math.radians(pose.heading); c, s = math.cos(a), math.sin(a)
    return [(pose.x + pixels_per_mm*(x*c-y*s), pose.y + pixels_per_mm*(x*s+y*c)) for x,y in polygon]


def _polygon(points):
    p = Polygon(points)
    if not p.is_valid or p.is_empty or p.area <= 0:
        raise ValueError('Invalid polygon: check footprint/object geometry')
    return p


def polygons_intersect(polygon_a, polygon_b):
    return _polygon(polygon_a).intersects(_polygon(polygon_b))


def sample_action_poses(start, action, pixels_per_mm, step_mm=5, step_deg=2, radius_mm=0):
    from planner import Pose, normalize_angle
    if min(step_mm, step_deg, pixels_per_mm) <= 0: raise ValueError('Sampling steps and scale must be positive')
    if isinstance(action, Turn):
        n = max(1, math.ceil(abs(action.angle)/step_deg), math.ceil(radius_mm*abs(math.radians(action.angle))/step_mm))
        for i in range(n+1): yield Pose(start.x,start.y,normalize_angle(start.heading+action.angle*i/n))
    elif isinstance(action,(Drive,LineTrack)):
        n = max(1, math.ceil(abs(action.distance)/step_mm)); a=math.radians(start.heading)
        for i in range(n+1):
            d=action.distance*pixels_per_mm*i/n
            yield Pose(start.x+d*math.cos(a),start.y+d*math.sin(a),start.heading)
    else: raise NotImplementedError(type(action).__name__)


@dataclass(frozen=True)
class Obstacle:
    name: str
    polygon: tuple


@dataclass(frozen=True)
class CollisionResult:
    obstacle: str
    pose: object
    action_index: int


class CollisionChecker:
    """Prepared obstacles reused for every candidate. Contact counts as collision.
    Sampling padding bounds between-sample point motion for nominal rigid actions.
    Does not model PID deviation, attachment articulation or intentional contact.
    """
    def __init__(self, footprint_mm, obstacles, pixels_per_mm, bounds=None, margin_mm=5, step_mm=5, step_deg=2):
        if min(pixels_per_mm,step_mm,step_deg)<=0 or margin_mm<0: raise ValueError('Invalid collision settings')
        self.footprint=tuple(footprint_mm); _polygon(self.footprint)
        self.scale=pixels_per_mm; self.step_mm=step_mm; self.step_deg=step_deg
        self.radius=max(math.hypot(x,y) for x,y in self.footprint)
        self.padding=(margin_mm+step_mm/2+1e-6)*pixels_per_mm
        self.obstacle_bounds=[]
        self.obstacles=[]
        for o in obstacles:
            polygon=_polygon(o.polygon).buffer(self.padding)
            self.obstacle_bounds.append(polygon.bounds)
            self.obstacles.append((o.name,prep(polygon)))
        self.bounds=prep(box(*bounds).buffer(-self.padding)) if bounds is not None else None

    def detour_waypoints(self):
        """Conservative corner hints, not a complete free-space search."""
        gap=self.radius*self.scale+max(1.0, self.step_mm*self.scale)
        for x0,y0,x1,y1 in self.obstacle_bounds:
            yield from ((x0-gap,y0-gap),(x0-gap,y1+gap),
                        (x1+gap,y0-gap),(x1+gap,y1+gap))

    def check_pose(self, pose, action_index=-1):
        p=_polygon(transform_polygon(self.footprint,pose,self.scale))
        if self.bounds is not None and not self.bounds.contains(p): return CollisionResult('Mat boundary',pose,action_index)
        for name, obstacle in self.obstacles:
            if obstacle.intersects(p): return CollisionResult(name,pose,action_index)
        return None

    def check_route(self,start,actions):
        pose=start
        hit=self.check_pose(pose)
        if hit: return hit
        for i,action in enumerate(actions):
            for pose in sample_action_poses(pose,action,self.scale,self.step_mm,self.step_deg,self.radius):
                hit=self.check_pose(pose,i)
                if hit: return hit
        return None


def check_pose_collision(pose, footprint_mm, obstacles, pixels_per_mm, **kwargs):
    return CollisionChecker(footprint_mm,obstacles,pixels_per_mm,**kwargs).check_pose(pose)


def check_route_collision(start, actions, footprint_mm, obstacles, pixels_per_mm, **kwargs):
    return CollisionChecker(footprint_mm,obstacles,pixels_per_mm,**kwargs).check_route(start,actions)
