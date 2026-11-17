import numpy as np
import math
from dataclasses import dataclass
from navigation import Node, Edge
from actions import Drive, Turn, LineTrack, TrackToIntersection
import copy


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class TrackCandidate:
    point: tuple
    edge: Edge
    segment_index: int
    kind: str


def closest_point_on_segment(point, start, end):
    point = np.array(point)
    start = np.array(start)
    end = np.array(end)
    vector_start_point = point - start
    if np.array_equal(start, end):
        return start
    vector_start_end = end - start
    t = np.dot(vector_start_end, vector_start_point) / np.dot(vector_start_end, vector_start_end)
    t = max(0, min(t, 1))
    return start + t * vector_start_end


def closest_point_on_graph(point, graph):
    closest_point_on_graph = None
    distance_to_closest_point = float('inf')
    closest_edge = None
    closest_segment_index = None
    for edge in graph.edges:
        for i in range(len(edge.points) - 1):
            candidate = closest_point_on_segment(point, edge.points[i], edge.points[i + 1])
            candidate_distance = math.dist(point, candidate)
            if distance_to_closest_point > candidate_distance:
                closest_point_on_graph = candidate
                distance_to_closest_point = candidate_distance
                closest_edge = edge
                closest_segment_index = i
    return (closest_point_on_graph, closest_edge, closest_segment_index, distance_to_closest_point)


def split_edge_points(edge, point, segment_index):
    new_edge_1 = edge.points[:segment_index + 1]
    new_edge_2 = edge.points[segment_index + 1:]
    point = tuple(point)
    if not np.allclose(new_edge_1[-1], point, rtol=0, atol=1e-06):
        new_edge_1 = new_edge_1 + (point,)
    if not np.allclose(new_edge_2[0], point, rtol=0, atol=1e-06):
        new_edge_2 = (point,) + new_edge_2
    return (new_edge_1, new_edge_2)


def polyline_length(points):
    total_length = 0
    for i in range(len(points) - 1):
        total_length += math.dist(points[i], points[i + 1])
    return total_length


def insert_node_on_edge(graph, edge, point, segment_index):
    if np.allclose(point, edge.points[0], rtol=0, atol=1e-06):
        return edge.start
    if np.allclose(point, edge.points[-1], rtol=0, atol=1e-06):
        return edge.end
    # Split only when the point is not already an endpoint.
    new_node_id = max(graph.nodes.keys()) + 1
    points_1, points_2 = split_edge_points(edge, point, segment_index)
    length_1 = polyline_length(points_1) / graph.pixels_per_mm
    length_2 = polyline_length(points_2) / graph.pixels_per_mm
    new_edge_1 = Edge(edge.start, new_node_id, points_1, length_1)
    new_edge_2 = Edge(new_node_id, edge.end, points_2, length_2)
    graph.nodes[new_node_id] = Node(new_node_id, point[0], point[1])
    graph.edges.remove(edge)
    graph.edges.append(new_edge_1)
    graph.edges.append(new_edge_2)
    return new_node_id


def normalize_angle(angle):
    return (angle + 180) % 360 - 180


def generate_direct_candidates(start, destination, pixels_per_mm):
    if pixels_per_mm <= 0:
        raise ValueError('pixels_per_mm must be positive')
    dx = destination[0] - start.x
    dy = destination[1] - start.y
    distance = math.hypot(dx, dy) / pixels_per_mm
    if distance < 1e-06:
        return [()]
    # Both candidates reach the same point with different final headings.
    forward_heading = math.degrees(math.atan2(dy, dx))
    reverse_heading = normalize_angle(forward_heading + 180)
    forward_turn = normalize_angle(forward_heading - start.heading)
    reverse_turn = normalize_angle(reverse_heading - start.heading)
    forward_actions = []
    reverse_actions = []
    if abs(forward_turn) > 1e-06:
        forward_actions.append(Turn(forward_turn))
    forward_actions.append(Drive(distance))
    if abs(reverse_turn) > 1e-06:
        reverse_actions.append(Turn(reverse_turn))
    reverse_actions.append(Drive(-distance))
    return [tuple(forward_actions), tuple(reverse_actions)]


def calculate_end_pose(start, actions, pixels_per_mm):
    if pixels_per_mm <= 0:
        raise ValueError('pixels_per_mm must be positive')
    x = start.x
    y = start.y
    heading = start.heading
    for action in actions:
        if isinstance(action, Turn):
            heading = normalize_angle(heading + action.angle)
        elif isinstance(action, (Drive, LineTrack)):
            distance = action.distance * pixels_per_mm
            radians = math.radians(heading)
            x += distance * math.cos(radians)
            y += distance * math.sin(radians)
        else:
            raise NotImplementedError(f'Unsupported action: {type(action).__name__}')
    return Pose(x, y, normalize_angle(heading))


def estimate_execution_time(actions, robot, drive_motor_dps=600.0, turn_motor_dps=350.0, drive_overhead_s=0.15, turn_overhead_s=0.2, line_motor_dps=400.0, line_overhead_s=0.2):
    if min(robot.wheel_diameter, robot.axle_track, drive_motor_dps, turn_motor_dps, line_motor_dps) <= 0:
        raise ValueError('Robot dimensions and speeds must be positive')
    wheel_circumference = math.pi * robot.wheel_diameter
    drive_speed = wheel_circumference * drive_motor_dps / 360
    turn_speed = robot.wheel_diameter * turn_motor_dps / robot.axle_track
    total_time = 0.0
    for action in actions:
        if isinstance(action, LineTrack):
            if action.distance != 0:
                total_time += abs(action.distance) / (wheel_circumference * line_motor_dps / 360) + line_overhead_s
        elif isinstance(action, Drive):
            if action.distance != 0:
                total_time += abs(action.distance) / drive_speed + drive_overhead_s
        elif isinstance(action, Turn):
            if action.angle != 0:
                total_time += abs(action.angle) / turn_speed + turn_overhead_s
        else:
            raise NotImplementedError(f'Unsupported action: {type(action).__name__}')
    return total_time


def evaluate_candidate(start, actions, robot, pixels_per_mm,
                       prefer_line_tracking=False, line_preference_s=1.8):
    """Draft cost (seconds-equivalent). No claims about measured reliability.

    Give a *bounded* preference to a meaningful continuous tracking run.
    A very short run earns nothing; many separate acquisitions do not stack
    unlimited bonuses. Longer or physically infeasible detours are not rewarded.
    """
    time_s = estimate_execution_time(actions, robot)
    if not prefer_line_tracking:
        return time_s
    runs = [abs(a.distance) for a in actions if isinstance(a, LineTrack)]
    useful = [d for d in runs if d >= 110.0]
    # First useful section supplies most value; other sections have diminishing
    # returns, preventing the planner from following the entire line network.
    benefit = min(float(line_preference_s),
                  sum(min(1.0, (d - 110.0) / 210.0) for d in useful) * 1.15)
    # A modest complexity penalty discourages the 34-action graph routes.
    complexity = 0.075 * max(0, len(actions) - 4)
    return time_s + complexity - benefit


def generate_detour_candidates(start, destination, pixels_per_mm,
                               collision_checker, max_waypoints=24,
                               max_pair_tests=48, max_results=24):
    """Try up to two obstacle-clearance corner waypoints.

    Obstacles with nonzero area often require TWO waypoints to go around;
    one corner alone is insufficient. Still a heuristic, not full A*.
    """
    a = (start.x, start.y)
    points = list(dict.fromkeys(collision_checker.detour_waypoints()))
    points.sort(key=lambda p: math.dist(a, p) + math.dist(p, destination))
    points = points[:max_waypoints]
    paths = [(p,) for p in points]
    pairs = [(math.dist(a,p)+math.dist(p,q)+math.dist(q,destination),(p,q))
             for p in points for q in points if p != q]
    pairs.sort(key=lambda item:item[0])
    paths.extend(pair for _,pair in pairs[:max_pair_tests])
    results = []
    seen = set()
    for waypoints in paths:
        beams = [((),start)]
        for point in (*waypoints, destination):
            next_beams = []
            for prefix, pose in beams:
                for leg in generate_direct_candidates(pose, point, pixels_per_mm):
                    if collision_checker.check_route(pose,leg) is not None:
                        continue
                    next_beams.append((prefix+leg,calculate_end_pose(pose,leg,pixels_per_mm)))
            if not next_beams:
                beams=[]
                break
            next_beams.sort(key=lambda pair:len(pair[0]))
            beams = next_beams[:8]
        for actions,_ in beams:
            actions=simplify_actions(actions)
            if actions not in seen and collision_checker.check_route(start,actions) is None:
                seen.add(actions)
                results.append(actions)
                if len(results)>=max_results:
                    return results
    return results


def simplify_actions(actions, epsilon_distance=0.5, epsilon_angle=0.25):
    """Remove tiny actions and combine *adjacent compatible* motions.

    Never merge across a turn, sensor reacquisition, or a different tracking side.
    This is mechanical simplification, not permission to cut corners or walls.
    """
    simplified = []
    for action in actions:
        if isinstance(action, Turn):
            if abs(action.angle) < epsilon_angle:
                continue
            if simplified and isinstance(simplified[-1], Turn):
                action = Turn(normalize_angle(simplified.pop().angle + action.angle))
                if abs(action.angle) < epsilon_angle:
                    continue
        elif isinstance(action, (Drive, LineTrack)):
            if abs(action.distance) < epsilon_distance:
                continue
            if simplified:
                prev = simplified[-1]
                same_drive = (isinstance(action, Drive) and isinstance(prev, Drive)
                              and action.method == prev.method
                              and action.distance * prev.distance > 0)
                same_track = (type(action) is LineTrack and type(prev) is LineTrack
                              and action.side == prev.side
                              and action.distance * prev.distance > 0)
                if same_drive:
                    simplified[-1] = Drive(prev.distance + action.distance, method=action.method)
                    continue
                if same_track:
                    simplified[-1] = LineTrack(prev.distance + action.distance, side=action.side)
                    continue
        simplified.append(action)
    return tuple(simplified)


def generate_l_shaped_candidates(start, destination, pixels_per_mm,
                                 collision_checker=None):
    """Axis-aligned doglegs, both orders, both forward/reverse choices.

    Works without a graph. Collision validation is optional but recommended.
    """
    corners = ((destination[0], start.y), (start.x, destination[1]))
    results, seen = [], set()
    for corner in corners:
        for first in generate_direct_candidates(start, corner, pixels_per_mm):
            middle = calculate_end_pose(start, first, pixels_per_mm)
            for second in generate_direct_candidates(middle, destination, pixels_per_mm):
                actions = simplify_actions(first + second)
                if actions in seen:
                    continue
                seen.add(actions)
                if collision_checker is None or collision_checker.check_route(start, actions) is None:
                    results.append(actions)
    return results


def generate_junction_corner_candidates(start, destination, base_routes,
                                        pixels_per_mm, collision_checker=None,
                                        max_results=160):
    """After a known intersection stop, continue with a simple orthogonal dogleg.

    In particular, this can give: TRACK_TO_INTERSECTION -> DRIVE along the
    incoming heading -> TURN(approximately 90) -> DRIVE toward destination.
    The chosen junction is a sensor stop, not a forced chassis pivot point.
    All geometry is nominal and candidates remain subject to collision checks.
    Original candidates are retained by the caller.
    """
    results, seen = [], set()
    for actions in base_routes:
        prefix = []
        pose = start
        for action in actions:
            prefix.append(action)
            pose = calculate_end_pose(pose, (action,), pixels_per_mm)
            if not isinstance(action, TrackToIntersection):
                continue
            # The two possible corner orders make it possible to drive some
            # distance beyond the junction before making the 90-degree turn.
            for departure in generate_l_shaped_candidates(
                    pose, destination, pixels_per_mm, collision_checker=None):
                candidate = simplify_actions(tuple(prefix) + departure)
                if candidate in seen or candidate == tuple(actions):
                    continue
                if collision_checker is not None and collision_checker.check_route(
                        start, candidate) is not None:
                    continue
                seen.add(candidate)
                results.append(candidate)
                if len(results) >= max_results:
                    return results
            # Only use the first junction stop: don't splice arbitrary later
            # actions after a new departure from an earlier landmark.
            break
    return results


def generate_shortcut_candidates(start, destination, hybrid_routes,
                                 pixels_per_mm, collision_checker=None,
                                 min_tracking_mm=80.0, max_shortcuts=240):
    """Leave a tracked route early, then finish with a direct movement.

    Only shortcuts AFTER useful tracking; keep original candidate intact.
    All shortcut geometry is validated against the collision checker.
    """
    results, seen = [], set()
    for original in hybrid_routes:
        prefix = []
        pose = start
        tracking_mm = 0.0
        for action in original[:-1]:
            prefix.append(action)
            pose = calculate_end_pose(pose, (action,), pixels_per_mm)
            if isinstance(action, LineTrack):
                tracking_mm += abs(action.distance)
            if not isinstance(action, LineTrack) or tracking_mm < min_tracking_mm:
                continue
            for departure in generate_direct_candidates(pose, destination, pixels_per_mm):
                candidate = simplify_actions(tuple(prefix) + departure)
                if candidate == original or candidate in seen:
                    continue
                seen.add(candidate)
                if collision_checker is None or collision_checker.check_route(start, candidate) is None:
                    results.append(candidate)
                    if len(results) >= max_shortcuts:
                        return results
    return results


def generate_opportunistic_track_candidates(start, destination, graph, robot,
                                            collision_checker=None,
                                            line_width_mm=20.0,
                                            edge_validator=None,
                                            min_track_mm=110.0,
                                            max_sections=36,
                                            max_results=120):
    """Use a useful straight *section* of a line, without following graph detours.

    Samples entry/exit points on individual straight graph edges, including
    projections from A and B. Both tracking sides and forward/reverse approaches
    are considered. Candidates are still nominal; final collision/edge checks
    remain mandatory. This is deliberately a bounded heuristic.
    """
    from sensor_geometry import generate_tracking_poses, generate_line_acquisition
    scale = graph.pixels_per_mm
    if scale <= 0 or max_sections <= 0:
        return []
    start_xy = (start.x, start.y)
    direct = math.dist(start_xy, destination) / scale
    if direct <= 1e-6:
        return []
    sections = []
    for edge in graph.edges:
        if len(edge.points) < 2:
            continue
        a, b = edge.points[0], edge.points[-1]
        dx, dy = b[0]-a[0], b[1]-a[1]
        length_px = math.hypot(dx, dy)
        if length_px < (min_track_mm + 25.0) * scale:
            continue
        # Ignore curved edges; a single LineTrack is only nominally straight.
        deviation = max(math.dist(p, closest_point_on_segment(p, a, b)) for p in edge.points)
        if deviation / scale > 4.0 or edge.length_mm > length_px / scale * 1.05:
            continue
        for u, v in ((a,b),(b,a)):
            ux = (v[0]-u[0])/length_px
            uy = (v[1]-u[1])/length_px
            # Prefer progress towards the destination rather than tracking away.
            progress = (destination[0]-start.x)*ux + (destination[1]-start.y)*uy
            if progress <= 0:
                continue
            def along(point):
                return max(0.0, min(1.0, ((point[0]-u[0])*ux+(point[1]-u[1])*uy)/length_px))
            entry_fracs = (0.0, along(start_xy), 0.20, 0.35)
            exit_fracs = (1.0, along(destination), 0.65, 0.80)
            for ef in entry_fracs:
                for xf in exit_fracs:
                    if (xf-ef)*length_px/scale < min_track_mm + 30.0:
                        continue
                    p0=(u[0]+ef*length_px*ux, u[1]+ef*length_px*uy)
                    p1=(u[0]+xf*length_px*ux, u[1]+xf*length_px*uy)
                    lower=(math.dist(start_xy,p0)+math.dist(p0,p1)+math.dist(p1,destination))/scale
                    # Keep modest detours, but allow more when the direct route
                    # is blocked. This is just a geometry shortlist.
                    if lower > direct*1.55 + 170:
                        continue
                    sections.append((lower,p0,p1))
    sections.sort(key=lambda item:item[0])
    seen_sections=set()
    results, seen_actions=[],set()
    for _,a,b in sections:
        key=(round(a[0],1),round(a[1],1),round(b[0],1),round(b[1],1))
        if key in seen_sections:
            continue
        seen_sections.add(key)
        if len(seen_sections)>max_sections:
            break
        for leg in generate_tracking_poses(a,b,robot,scale,line_width_mm):
            if leg.distance_mm < min_track_mm:
                continue
            if edge_validator is not None and not edge_validator(leg):
                continue
            for approach in generate_line_acquisition(start,leg.start,scale):
                if collision_checker is not None and collision_checker.check_route(start,approach) is not None:
                    continue
                track_action=LineTrack(float(leg.distance_mm),side=leg.side)
                if collision_checker is not None and collision_checker.check_route(leg.start,(track_action,)) is not None:
                    continue
                for departure in generate_direct_candidates(leg.end,destination,scale):
                    if collision_checker is not None and collision_checker.check_route(leg.end,departure) is not None:
                        continue
                    candidate=simplify_actions(approach+(track_action,)+departure)
                    if candidate in seen_actions:
                        continue
                    if collision_checker is not None and collision_checker.check_route(start,candidate) is not None:
                        continue
                    seen_actions.add(candidate)
                    results.append(candidate)
                    if len(results)>=max_results:
                        return results
    return results




def junction_node_ids(graph):
    """Graph branch nodes, not arbitrary corners or endpoints.

    A physical junction may be missed by image extraction; this conservative
    heuristic emits no intersection-stop command unless >=3 edges meet.
    """
    degree = {node_id: 0 for node_id in graph.nodes}
    for edge in graph.edges:
        if edge.start == edge.end:
            degree[edge.start] += 2
        else:
            degree[edge.start] += 1
            degree[edge.end] += 1
    return {node_id for node_id, d in degree.items() if d >= 3}


def _track_legs_for_section(a, b, robot, scale, width_mm,
                            junction_end=False, inset_mm=15.0):
    """Nominal straight sensor-edge poses; optionally stop at the end junction.

    On a junction-ending leg, only the entry is inset. In particular, we do NOT
    shorten the end 15-20 mm away from the actual printed junction.
    """
    from sensor_geometry import (generate_tracking_poses, get_line_edge_point,
                                 sensor_to_robot_pose, TrackingLeg)
    if not junction_end:
        return [(leg, LineTrack(float(leg.distance_mm), side=leg.side))
                for leg in generate_tracking_poses(a, b, robot, scale, width_mm)]
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = math.hypot(dx, dy)
    inset = max(inset_mm, width_mm / 2 + getattr(robot, 'sensor_diameter_mm', 0) / 2) * scale
    if length <= inset + 1e-6:
        return []
    unit = (dx/length, dy/length)
    p = (a[0] + unit[0]*inset, a[1] + unit[1]*inset)
    heading = math.degrees(math.atan2(dy, dx))
    result = []
    for side in ('left', 'right'):
        sp = get_line_edge_point(p, a, b, side, width_mm, scale)
        sq = get_line_edge_point(b, a, b, side, width_mm, scale)
        args = (heading, robot.sensor_x_mm, robot.sensor_y_mm, scale)
        start = sensor_to_robot_pose(sp, *args)
        end = sensor_to_robot_pose(sq, *args)
        distance_mm = (length - inset) / scale
        leg = TrackingLeg(start, end, distance_mm, side)
        result.append((leg, TrackToIntersection(float(distance_mm), side=side)))
    return result


def _validate_junction_leg(leg, robot, scale, width_mm, edge_validator):
    """Validate its ordinary approach section; terminal crossing is different.

    Raster validators require pure black/white edge at both endpoints, which is
    typically false AT a T/cross junction. Test a prefix ending before it.
    """
    if edge_validator is None:
        return True
    from sensor_geometry import TrackingLeg
    inset = max(18.0, width_mm + getattr(robot,'sensor_diameter_mm',0))
    length = leg.distance_mm - inset
    if length < 60:
        return False
    theta=math.radians(leg.start.heading)
    end=Pose(leg.start.x+length*scale*math.cos(theta),
             leg.start.y+length*scale*math.sin(theta),leg.start.heading)
    return edge_validator(TrackingLeg(leg.start,end,length,leg.side))


def _junction_entry_points(a, b, start, destination, pixels_per_mm,
                           min_track_mm=100.0, max_entries=6):
    """Suggest entrances *inside* a straight line ending at junction b.

    All coordinates are image pixels. The nominal tracking distance is
    measured from the chosen entrance to b, not from the original graph node.
    Keep the full-edge entrance as a fallback, but sample projected and
    intermediate entrances so the robot need not backtrack to the line start.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return []
    ux, uy = dx / length, dy / length
    # Reserve space for the entry inset and a meaningful tracking interval.
    max_fraction = 1.0 - (min_track_mm + 28.0) * pixels_per_mm / length
    if max_fraction <= 0:
        return [tuple(a)]
    max_fraction = min(max_fraction, 0.93)

    def projection(point):
        t = ((point[0] - a[0]) * ux + (point[1] - a[1]) * uy) / length
        return max(0.0, min(max_fraction, t))

    origin = (start.x, start.y)
    start_t = projection(origin)
    destination_t = projection(destination)
    fractions = [0.0, start_t, destination_t,
                 0.25 * max_fraction, 0.50 * max_fraction,
                 0.75 * max_fraction, max_fraction]
    fractions = sorted(set(round(t, 6) for t in fractions),
                      key=lambda t: (math.dist(origin,
                              (a[0] + t * dx, a[1] + t * dy))
                             + 0.55 * (1.0 - t) * length,
                             -t))
    chosen = []
    for t in fractions:
        point = (a[0] + t * dx, a[1] + t * dy)
        if all(math.dist(point, old) > 8.0 * pixels_per_mm for old in chosen):
            chosen.append(point)
        if len(chosen) >= max_entries:
            break
    return chosen


def generate_intersection_tracking_candidates(start, destination, graph, robot,
                                               collision_checker=None,
                                               line_width_mm=20.0,
                                               edge_validator=None,
                                               min_track_mm=100.0,
                                               max_results=80):
    """Track a straight line to a graph-verified junction, entering mid-line.

    Unlike earlier versions, the robot does not have to acquire the line near
    its original graph endpoint. Try direct projection and several interior
    positions, retaining the original entry as a fallback. The downstream
    intersection is unchanged, so the sensor-stop command remains valid.
    """
    from sensor_geometry import generate_line_acquisition
    scale = graph.pixels_per_mm
    junctions = junction_node_ids(graph)
    result, seen = [], set()
    sections = []
    for edge in graph.edges:
        if len(edge.points) < 2:
            continue
        a, b = edge.points[0], edge.points[-1]
        chord = math.dist(a, b)
        if chord < max(min_track_mm + 25.0, 120.0) * scale:
            continue
        if max(math.dist(p, closest_point_on_segment(p, a, b))
               for p in edge.points) > 4.0 * scale:
            continue
        for node_id, u, v in ((edge.end, a, b),
                              (edge.start, b, a)):
            if node_id not in junctions:
                continue
            for entry_point in _junction_entry_points(
                    u, v, start, destination, scale, min_track_mm):
                remaining = math.dist(entry_point, v) / scale
                if remaining < min_track_mm + 20.0:
                    continue
                # Prefer a short approach and a useful, but not excessive,
                # section of tracking. This is only a generation ordering.
                score = (math.dist((start.x, start.y), entry_point)
                         + 0.55 * math.dist(entry_point, v)
                         + math.dist(v, destination)) / scale
                sections.append((score, entry_point, v))
    sections.sort(key=lambda s: s[0])
    for _, a, b in sections[:max_results * 8]:
        for leg, track_action in _track_legs_for_section(
                a, b, robot, scale, line_width_mm, True):
            if leg.distance_mm < min_track_mm or not _validate_junction_leg(
                    leg, robot, scale, line_width_mm, edge_validator):
                continue
            for approach in generate_line_acquisition(start, leg.start, scale):
                if collision_checker is not None and collision_checker.check_route(
                        start, approach) is not None:
                    continue
                if collision_checker is not None and collision_checker.check_route(
                        leg.start, (track_action,)) is not None:
                    continue
                for departure in generate_direct_candidates(
                        leg.end, destination, scale):
                    candidate = simplify_actions(approach + (track_action,) + departure)
                    if candidate in seen:
                        continue
                    if collision_checker is not None and collision_checker.check_route(
                            start, candidate) is not None:
                        continue
                    seen.add(candidate)
                    result.append(candidate)
                    if len(result) >= max_results:
                        return result
    return result

def augment_route_with_tracking(start, actions, graph, robot,
                                collision_checker=None,
                                line_width_mm=20.0,
                                edge_validator=None,
                                min_track_mm=110.0,
                                max_results=48):
    """Replace ONE Drive in an already-simple route with an optional line leg.

    Preserve its exact end position AND heading, hence preserve subsequent
    movements and collision checks. The base route is never removed.
    """
    from sensor_geometry import generate_line_acquisition
    scale=graph.pixels_per_mm
    junctions=junction_node_ids(graph)
    result=[];seen=set()
    pose=start
    for action_index, action in enumerate(actions):
        end=calculate_end_pose(pose,(action,),scale)
        if not isinstance(action,Drive) or abs(action.distance)<min_track_mm:
            pose=end;continue
        a0=(pose.x,pose.y);a1=(end.x,end.y)
        ax,ay=a1[0]-a0[0],a1[1]-a0[1];length=math.hypot(ax,ay)
        if length<1e-6:pose=end;continue
        ux,uy=ax/length,ay/length
        options=[]
        for edge in graph.edges:
            if len(edge.points)<2:continue
            for u,v,end_node in ((edge.points[0],edge.points[-1],edge.end),
                                 (edge.points[-1],edge.points[0],edge.start)):
                bx,by=v[0]-u[0],v[1]-u[1]; bl=math.hypot(bx,by)
                if bl<max(100.0,min_track_mm)*scale:continue
                cosine=(ux*bx+uy*by)/bl
                if cosine<math.cos(math.radians(9)):continue
                if max(math.dist(p,closest_point_on_segment(p,u,v)) for p in edge.points)>4*scale:continue
                # Signed position of each track endpoint along the original Drive.
                tu=((u[0]-a0[0])*ux+(u[1]-a0[1])*uy)/length
                tv=((v[0]-a0[0])*ux+(v[1]-a0[1])*uy)/length
                enter=max(0.0,tu); leave=min(1.0,tv)
                if leave<=enter:continue
                # The track should be close to the base corridor; offset due
                # to the light sensor can be significant but should be bounded.
                cross=abs((u[0]-a0[0])*uy-(u[1]-a0[1])*ux)/scale
                if cross>120:continue
                s=max(0,(enter-tu)*length/bl)
                t=min(1,(leave-tu)*length/bl)
                if t<=s:continue
                p=(u[0]+s*bx,u[1]+s*by)
                q=(u[0]+t*bx,u[1]+t*by)
                if math.dist(p,q)/scale<min_track_mm+25:continue
                true_junction=end_node in junctions and abs(t-1)<1e-6
                score=(math.dist(a0,p)+math.dist(q,a1))/scale+cross*.3
                options.append((score,p,q,true_junction))
        options.sort(key=lambda z:z[0]); keys=set()
        for _,p,q,is_junction in options[:20]:
            key=tuple(round(v,1) for v in (*p,*q))+(is_junction,)
            if key in keys:continue
            keys.add(key)
            for leg,track in _track_legs_for_section(p,q,robot,scale,line_width_mm,is_junction):
                if leg.distance_mm<min_track_mm:continue
                if is_junction:
                    if not _validate_junction_leg(leg,robot,scale,line_width_mm,edge_validator):continue
                elif edge_validator is not None and not edge_validator(leg):continue
                for approach in generate_line_acquisition(pose,leg.start,scale):
                    if collision_checker is not None and collision_checker.check_route(pose,approach) is not None:continue
                    if collision_checker is not None and collision_checker.check_route(leg.start,(track,)) is not None:continue
                    for departure in generate_direct_candidates(leg.end,a1,scale):
                        dep_end=calculate_end_pose(leg.end,departure,scale)
                        correction=normalize_angle(end.heading-dep_end.heading)
                        tail=departure+((Turn(correction),) if abs(correction)>0.25 else ())
                        prefix=tuple(actions[:action_index])
                        suffix=tuple(actions[action_index+1:])
                        candidate=simplify_actions(prefix+approach+(track,)+tail+suffix)
                        if candidate in seen:continue
                        try:
                            final=calculate_end_pose(start,candidate,scale)
                            original_final=calculate_end_pose(start,actions,scale)
                            if math.dist((final.x,final.y),(original_final.x,original_final.y))>0.5*scale:continue
                            if abs(normalize_angle(final.heading-original_final.heading))>0.5:continue
                        except (ValueError,NotImplementedError):continue
                        if collision_checker is not None and collision_checker.check_route(start,candidate) is not None:continue
                        seen.add(candidate);result.append(candidate)
                        if len(result)>=max_results:return result
        pose=end
    return result

def generate_navigation_alternatives(start, destination, robot, pixels_per_mm,
                                     graph=None, collision_checker=None,
                                     max_pairs=60, min_track_mm=40.0,
                                     prefer_line_tracking=True, line_preference_s=1.8,
                                     **hybrid_options):
    """Return feasible, labelled alternatives for the GUI; no fake diversity.

    Categories include Forward, Reverse, L-shaped, Detour, Hybrid, Shortcut, and Line-augmented.
    The ranking remains time-only until the user's new cost model is added.
    """
    if graph is not None and not math.isclose(pixels_per_mm, graph.pixels_per_mm, rel_tol=1e-9):
        raise ValueError('Planner and graph scales must match')
    alternatives, seen = [], set()
    def add(actions, label):
        actions = simplify_actions(actions)
        if actions in seen:
            return
        seen.add(actions)
        if collision_checker is not None and collision_checker.check_route(start, actions) is not None:
            return
        alternatives.append((label, actions))

    for direct in generate_direct_candidates(start, destination, pixels_per_mm):
        label = 'Reverse' if any(isinstance(a, Drive) and a.distance < 0 for a in direct) else 'Forward'
        add(direct, label)
    for route in generate_l_shaped_candidates(start, destination, pixels_per_mm, collision_checker):
        add(route, 'L-shaped')
    # Detours are never contingent on the hybrid generator failing.
    if collision_checker is not None:
        for route in generate_detour_candidates(start, destination, pixels_per_mm, collision_checker):
            add(route, 'Detour')
    if graph is not None:
        # Augment the SIMPLE geometric routes rather than forcing the graph to
        # dictate every physical movement; keep the originals alongside them.
        simple_routes = [actions for label,actions in alternatives
                         if label in ('Forward','Reverse','L-shaped','Detour')]
        for base in sorted(simple_routes, key=lambda r: estimate_execution_time(r,robot))[:12]:
            for route in augment_route_with_tracking(
                start,base,graph,robot,collision_checker=collision_checker,
                line_width_mm=hybrid_options.get('line_width_mm',20.0),
                edge_validator=hybrid_options.get('edge_validator'),
                min_track_mm=max(100.0,min_track_mm), max_results=12):
                add(route,'Line-augmented')
        # Generate nominal *intersection-stopping* alternatives independently.
        for route in generate_intersection_tracking_candidates(
            start,destination,graph,robot,collision_checker=collision_checker,
            line_width_mm=hybrid_options.get('line_width_mm',20.0),
            edge_validator=hybrid_options.get('edge_validator'),
            min_track_mm=max(100.0,min_track_mm)):
            add(route,'Intersection track')
        # The junction does not force an immediate diagonal departure.
        # Generate the practical alternative: drive further along the current
        # axis, turn once at a convenient corner, then approach the target.
        landmark_routes = [actions for label, actions in alternatives
                           if label in ('Line-augmented', 'Intersection track')
                           and any(isinstance(a, TrackToIntersection) for a in actions)]
        for route in generate_junction_corner_candidates(
                start, destination, landmark_routes, pixels_per_mm,
                collision_checker=collision_checker, max_results=160):
            add(route, 'Junction corner')
        # Generate useful single-line sections before exploring full network paths.
        # This avoids an over-reliance on graph junctions and offers simple routes.
        for route in generate_opportunistic_track_candidates(
            start, destination, graph, robot, collision_checker=collision_checker,
            line_width_mm=hybrid_options.get('line_width_mm', 20.0),
            edge_validator=hybrid_options.get('edge_validator'),
            min_track_mm=max(110.0, min_track_mm)):
            add(route, 'Opportunistic track')
        hybrid = generate_hybrid_candidates(
            start, destination, graph, robot,
            max_pairs=max_pairs, min_track_mm=min_track_mm,
            collision_checker=collision_checker, **hybrid_options)
        for route in hybrid:
            add(route, 'Hybrid')
        for route in generate_shortcut_candidates(start, destination, hybrid,
                                                  pixels_per_mm, collision_checker):
            add(route, 'Shortcut')
    alternatives.sort(key=lambda item: evaluate_candidate(
        start, item[1], robot, pixels_per_mm,
        prefer_line_tracking=prefer_line_tracking,
        line_preference_s=line_preference_s))
    return alternatives


def plan_navigation(start, destination, robot, pixels_per_mm, graph=None, collision_checker=None, **hybrid_options):
    alternatives = generate_navigation_alternatives(
        start, destination, robot, pixels_per_mm,
        graph=graph, collision_checker=collision_checker, **hybrid_options)
    if not alternatives:
        return None, float('inf')
    best = alternatives[0][1]
    return best, evaluate_candidate(
        start, best, robot, pixels_per_mm,
        prefer_line_tracking=hybrid_options.get('prefer_line_tracking', True),
        line_preference_s=hybrid_options.get('line_preference_s', 1.8))


def segment_intersection(a, b, c, d):
    a = np.array(a, dtype=float)
    b = np.array(b, dtype=float)
    c = np.array(c, dtype=float)
    d = np.array(d, dtype=float)
    r = b - a
    s = d - c
    denominator = r[0] * s[1] - r[1] * s[0]
    # Parallel or collinear segments.
    if abs(denominator) < 1e-09:
        return None
    difference = c - a
    t = (difference[0] * s[1] - difference[1] * s[0]) / denominator
    u = (difference[0] * r[1] - difference[1] * r[0]) / denominator
    if 0 <= t <= 1 and 0 <= u <= 1:
        return tuple(a + t * r)
    return None


def closest_point_on_edge(point, edge):
    closest_point = None
    closest_segment_index = None
    closest_distance = float('inf')
    for i in range(len(edge.points) - 1):
        candidate = closest_point_on_segment(point, edge.points[i], edge.points[i + 1])
        distance = math.dist(point, candidate)
        if distance < closest_distance:
            closest_point = tuple(candidate)
            closest_segment_index = i
            closest_distance = distance
    return (closest_point, closest_segment_index, closest_distance)


def generate_entry_candidates(start, destination, graph):
    start_point = (start.x, start.y)
    candidates = []
    for edge in graph.edges:
        if len(edge.points) < 2:
            continue
        for i in range(len(edge.points) - 1):
            point = segment_intersection(start_point, destination, edge.points[i], edge.points[i + 1])
            if point is not None:
                candidates.append(TrackCandidate(point, edge, i, 'direct_intersection'))
        candidates.append(TrackCandidate(edge.points[0], edge, 0, 'node'))
        candidates.append(TrackCandidate(edge.points[-1], edge, len(edge.points) - 2, 'node'))
        point, segment_index, distance = closest_point_on_edge(start_point, edge)
        if point is not None:
            candidates.append(TrackCandidate(point, edge, segment_index, 'projection'))
    return candidates


def generate_exit_candidates(entry, destination, graph):
    candidates = []
    for edge in graph.edges:
        if len(edge.points) < 2:
            continue
        candidates.append(TrackCandidate(edge.points[0], edge, 0, 'node'))
        candidates.append(TrackCandidate(edge.points[-1], edge, len(edge.points) - 2, 'node'))
        point, segment_index, distance = closest_point_on_edge(destination, edge)
        if point is not None:
            candidates.append(TrackCandidate(point, edge, segment_index, 'projection'))
    return candidates


def find_track_route(entry, exit, graph):
    graph_copy = copy.copy(graph)
    graph_copy.nodes = graph.nodes.copy()
    graph_copy.edges = graph.edges.copy()
    entry_edge_index = graph.edges.index(entry.edge)
    entry_edge = graph_copy.edges[entry_edge_index]
    exit_edge_index = graph.edges.index(exit.edge)
    exit_edge = graph_copy.edges[exit_edge_index]
    exit_segment_index = exit.segment_index
    entry_id = insert_node_on_edge(graph_copy, entry_edge, entry.point, entry.segment_index)
    if exit_edge not in graph_copy.edges:
        replacement_edges = graph_copy.edges[-2:]
        closest_distance = float('inf')
        for edge in replacement_edges:
            point, segment_index, distance = closest_point_on_edge(exit.point, edge)
            if distance < closest_distance:
                closest_distance = distance
                exit_edge = edge
                exit_segment_index = segment_index
        if closest_distance > 1e-6:
            raise ValueError('Exit point not on replacement edges')
    exit_id = insert_node_on_edge(graph_copy, exit_edge, exit.point, exit_segment_index)
    if entry_id == exit_id:
        return []
    return graph_copy.shortest_path(start=entry_id, end=exit_id)


def track_route_to_actions(start, route, robot, pixels_per_mm, side='right', straight_tolerance_mm=5.0):
    """Ideal centreline prototype. Reject curved edges rather than invent turns.
    The caller must arrange actual sensor acquisition and corner manoeuvres.
    """
    if pixels_per_mm <= 0:
        raise ValueError('pixels_per_mm must be positive')
    actions = []
    pose = start
    for edge, reversed_edge in route:
        points = edge.points[::-1] if reversed_edge else edge.points
        a, b = points[0], points[-1]
        if math.dist((pose.x, pose.y), a) > 1e-5:
            raise ValueError('Track route is not continuous')
        distance = math.dist(a, b) / pixels_per_mm
        if distance < 1e-6:
            raise ValueError('Degenerate or closed tracking edge')
        deviation = max(math.dist(p, closest_point_on_segment(p, a, b)) for p in points) / pixels_per_mm
        if deviation > straight_tolerance_mm or edge.length_mm > distance * 1.15:
            raise ValueError('Curved edge requires a sensor-aware motion model')
        target_heading = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
        turn = normalize_angle(target_heading - pose.heading)
        if abs(turn) > 1e-6:
            actions.append(Turn(float(turn)))
        # Chord distance keeps nominal end pose consistent with geometry.
        actions.append(LineTrack(float(distance), side=side))
        pose = Pose(b[0], b[1], target_heading)
    return tuple(actions)


def _unique_track_candidates(candidates):
    seen = set()
    for candidate in candidates:
        key = (id(candidate.edge), round(float(candidate.point[0]), 6), round(float(candidate.point[1]), 6))
        if key not in seen:
            seen.add(key)
            yield candidate


def _midline_hybrid_route_variants(start, route, pixels_per_mm,
                                   min_track_mm=40.0):
    """Return original route plus one starting *inside* its first track edge.

    Track routes are (edge, reversed_traversal) pairs. Projection is made onto
    the oriented first polyline; its tail is retained so all later edges and
    the exit destination stay connected. This does not mutate the graph.
    The acquisition pose and side are recalculated by the sensor-aware planner.
    """
    variants = [route]
    if not route or pixels_per_mm <= 0:
        return variants

    edge, reversed_traversal = route[0]
    points = tuple(reversed(edge.points)) if reversed_traversal else edge.points
    if len(points) < 2:
        return variants
    start_xy = (start.x, start.y)
    closest, index, distance = closest_point_on_edge(start_xy,
        Edge(edge.start, edge.end, points, edge.length_mm))
    if closest is None:
        return variants

    # Never turn a short tracking section into an even shorter acquisition.
    # Sensor-aware generation already insets its endpoints by ~15-20 mm.
    tail = (closest,) + tuple(points[index + 1:])
    if len(tail) < 2:
        return variants
    remaining_mm = polyline_length(tail) / pixels_per_mm
    skipped_mm = (polyline_length(points) - polyline_length(tail)) / pixels_per_mm
    if skipped_mm < 20.0 or remaining_mm < max(100.0, min_track_mm + 40.0):
        return variants

    # A projection near a kink does not create a legitimate straight track.
    if max(math.dist(p, closest_point_on_segment(p, tail[0], tail[-1]))
           for p in tail) / pixels_per_mm > 5.0:
        return variants

    stored_points = tuple(reversed(tail)) if reversed_traversal else tail
    shortened = Edge(edge.start, edge.end, stored_points, remaining_mm)
    variants.append([(shortened, reversed_traversal)] + list(route[1:]))
    return variants


def _hybrid_pair_shortlist(pairs, max_pairs):
    """Reserve search slots for projected/interior entries, not just nodes.

    A purely distance-sorted shortlist often fills with almost-identical
    endpoint choices before it reaches a useful mid-line acquisition.
    """
    if max_pairs <= 0:
        return []
    pairs.sort(key=lambda item: item[0])
    projection_quota = max(1, max_pairs // 2)
    selected = []
    used = set()

    def add(item):
        _, entry, exit = item
        key = (id(entry.edge), round(float(entry.point[0]), 4),
               round(float(entry.point[1]), 4), id(exit.edge),
               round(float(exit.point[0]), 4), round(float(exit.point[1]), 4))
        if key in used:
            return
        used.add(key)
        selected.append(item)

    for pair in pairs:
        if pair[1].kind in ('projection', 'direct_intersection'):
            add(pair)
            if len(selected) >= projection_quota:
                break
    for pair in pairs:
        if len(selected) >= max_pairs:
            break
        add(pair)
    return selected


def generate_hybrid_candidates(start, destination, graph, robot, max_pairs=60,
                               min_track_mm=40.0, sensor_aware=None,
                               line_width_mm=20.0, edge_validator=None,
                               collision_checker=None, beam_width=4):
    """Hybrid routes with interior line acquisition and bounded pair search.

    Generate entry/exit points along edges, not just graph nodes. Besides
    splitting at those points, offer a route with its first edge trimmed to
    the robot's nearest practical mid-line entry. Both versions still go
    through the existing sensor-offset and collision checks.
    """
    scale = graph.pixels_per_mm
    if scale <= 0:
        raise ValueError('pixels_per_mm must be positive')
    if math.dist((start.x, start.y), destination) < 1e-6:
        return []
    if sensor_aware is None:
        sensor_aware = hasattr(robot, 'sensor_x_mm') and hasattr(robot, 'sensor_y_mm')

    entries = list(_unique_track_candidates(
        generate_entry_candidates(start, destination, graph)))
    if not entries:
        return []
    exits = list(_unique_track_candidates(
        generate_exit_candidates(entries[0], destination, graph)))
    pairs = []
    for entry in entries:
        for exit in exits:
            tracking_lower_bound = math.dist(entry.point, exit.point)
            if tracking_lower_bound / scale < min_track_mm:
                continue
            lower_bound = (math.dist((start.x, start.y), entry.point)
                           + tracking_lower_bound
                           + math.dist(exit.point, destination))
            pairs.append((lower_bound, entry, exit))

    candidates, seen = [], set()
    for _, entry, exit in _hybrid_pair_shortlist(pairs, max_pairs):
        try:
            route = find_track_route(entry, exit, graph)
            if not route:
                continue
            route_variants = _midline_hybrid_route_variants(
                start, route, scale, min_track_mm) if sensor_aware else [route]
            for path in route_variants:
                if sensor_aware:
                    options = sensor_aware_route_candidates(
                        start, destination, path, robot, scale, line_width_mm,
                        edge_validator, collision_checker, beam_width, min_track_mm)
                else:
                    options = []
                    for approach in generate_direct_candidates(
                            start, path[0][0].points[-1] if path[0][1]
                            else path[0][0].points[0], scale):
                        entry_pose = calculate_end_pose(start, approach, scale)
                        tracking = track_route_to_actions(entry_pose, path, robot, scale)
                        exit_pose = calculate_end_pose(entry_pose, tracking, scale)
                        for departure in generate_direct_candidates(
                                exit_pose, destination, scale):
                            options.append(approach + tracking + departure)
                for actions in options:
                    actions = simplify_actions(actions)
                    if actions not in seen:
                        seen.add(actions)
                        candidates.append(actions)
        except ValueError:
            # Disconnected, blocked, or unsupported line geometry.
            continue
    return candidates


def sensor_aware_route_candidates(start, destination, route, robot, scale, line_width_mm=20, edge_validator=None, collision_checker=None, beam_width=4, min_track_mm=40):
    """Build both edge sides per leg; retain a bounded beam of feasible prefixes.
    Pose acquisition and corner transitions are nominal dead reckoning.
    """
    from sensor_geometry import generate_tracking_poses, generate_line_acquisition
    if beam_width < 1: raise ValueError('beam_width must be positive')
    beam = [((), start, 0.0)]
    # Merge successive collinear graph edges into a single tracking leg.
    # A graph junction does not necessarily require a physical stop or turn.
    spans = []
    for edge, reversed_edge in route:
        points = edge.points[::-1] if reversed_edge else edge.points
        a, b = points[0], points[-1]
        # Check the actual geometry of each edge, not a stale edge from
        # another span. Curved pieces cannot be a straight LineTrack action.
        if max(math.dist(p, closest_point_on_segment(p, a, b)) for p in points) / scale > 5:
            return []
        if spans:
            prev_a, prev_b = spans[-1]
            ux,uy = prev_b[0]-prev_a[0],prev_b[1]-prev_a[1]
            vx,vy = b[0]-a[0],b[1]-a[1]
            denominator=math.hypot(ux,uy)*math.hypot(vx,vy)
            cosine=(ux*vx+uy*vy)/denominator if denominator>1e-9 else -1
            deviation=abs(ux*(b[1]-prev_a[1])-uy*(b[0]-prev_a[0]))/max(math.hypot(ux,uy),1e-9)
            if (math.dist(prev_b,a)<1e-5 and cosine>math.cos(math.radians(2))
                    and deviation/scale<=3):
                # Keep a single long leg only when it passes line-image checks.
                merged = generate_tracking_poses(prev_a,b,robot,scale,line_width_mm)
                if merged and (edge_validator is None or any(edge_validator(leg) for leg in merged)):
                    spans[-1]=(prev_a,b)
                    continue
        spans.append((a,b))
    for a,b in spans:
        chord = math.dist(a,b)/scale
        if chord <= 1e-6:
            return []
        legs = generate_tracking_poses(a,b,robot,scale,line_width_mm)
        legs = [leg for leg in legs if edge_validator is None or edge_validator(leg)]
        # Very short graph edges often occur near corners. Skip tracking on them;
        # the next leg's approach bridges the gap using direct movements.
        if not legs:
            return []
        following = []
        for prefix, pose, tracked in beam:
            for leg in legs:
                for approach in generate_line_acquisition(pose,leg.start,scale):
                    chunk = approach + (LineTrack(float(leg.distance_mm),side=leg.side),)
                    if collision_checker is not None and collision_checker.check_route(pose,chunk):
                        continue
                    end = calculate_end_pose(pose,chunk,scale)
                    following.append((prefix+chunk,end,tracked+leg.distance_mm))
        if not following: return []
        following.sort(key=lambda item: estimate_execution_time(item[0],robot))
        beam = following[:beam_width]
    result=[]
    for prefix,pose,tracked in beam:
        if tracked < min_track_mm: continue
        for departure in generate_direct_candidates(pose,destination,scale):
            if collision_checker is not None and collision_checker.check_route(pose,departure): continue
            result.append(prefix+departure)
    return result
