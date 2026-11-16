import numpy as np
import math
from dataclasses import dataclass
from navigation import Node, Edge
from actions import Drive, Turn, LineTrack
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


def evaluate_candidate(start, actions, robot, pixels_per_mm):
    return estimate_execution_time(actions, robot)


def plan_navigation(start, destination, robot, pixels_per_mm, graph=None, collision_checker=None, **hybrid_options):
    cost = float('inf')
    best_candidate = None
    candidates = generate_direct_candidates(start, destination, pixels_per_mm)
    if graph is not None:
        if not math.isclose(pixels_per_mm, graph.pixels_per_mm, rel_tol=1e-09):
            raise ValueError('Planner and graph scales must match')
        candidates.extend(generate_hybrid_candidates(start, destination, graph, robot, collision_checker=collision_checker, **hybrid_options))
    for candidate in candidates:
        if collision_checker is not None and collision_checker.check_route(start, candidate):
            continue
        current_cost = evaluate_candidate(start, candidate, robot, pixels_per_mm)
        if current_cost < cost:
            cost = current_cost
            best_candidate = candidate
    return (best_candidate, cost)


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


def generate_hybrid_candidates(start, destination, graph, robot, max_pairs=60, min_track_mm=40.0, sensor_aware=None, line_width_mm=20.0, edge_validator=None, collision_checker=None, beam_width=4):
    """Rank entry/exit pairs by geometric lower bound; test a bounded shortlist.
    Returns action tuples, not a claim of physical feasibility.
    """
    scale = graph.pixels_per_mm
    if scale <= 0:
        raise ValueError('pixels_per_mm must be positive')
    if math.dist((start.x, start.y), destination) < 1e-6:
        return []
    if sensor_aware is None:
        sensor_aware = hasattr(robot, "sensor_x_mm") and hasattr(robot, "sensor_y_mm")
    entries = list(_unique_track_candidates(generate_entry_candidates(start, destination, graph)))
    if not entries:
        return []
    exits = list(_unique_track_candidates(generate_exit_candidates(entries[0], destination, graph)))
    pairs = []
    for entry in entries:
        for exit in exits:
            tracking_lower_bound = math.dist(entry.point, exit.point)
            if tracking_lower_bound / scale < min_track_mm:
                continue
            lower_bound = math.dist((start.x,start.y), entry.point) + tracking_lower_bound + math.dist(exit.point,destination)
            pairs.append((lower_bound, entry, exit))
    pairs.sort(key=lambda item: item[0])
    candidates = []
    seen = set()
    for _, entry, exit in pairs[:max_pairs]:
        try:
            route = find_track_route(entry, exit, graph)
            if not route:
                continue
            if sensor_aware:
                for actions in sensor_aware_route_candidates(start, destination, route, robot, scale, line_width_mm, edge_validator, collision_checker, beam_width, min_track_mm):
                    if actions not in seen:
                        seen.add(actions)
                        candidates.append(actions)
                continue
            for approach in generate_direct_candidates(start, entry.point, scale):
                entry_pose = calculate_end_pose(start, approach, scale)
                tracking = track_route_to_actions(entry_pose, route, robot, scale)
                exit_pose = calculate_end_pose(entry_pose, tracking, scale)
                for departure in generate_direct_candidates(exit_pose, destination, scale):
                    actions = approach + tracking + departure
                    if actions not in seen:
                        seen.add(actions)
                        candidates.append(actions)
        except ValueError:
            # Disconnected or geometrically unsupported pair.
            continue
    return candidates


def sensor_aware_route_candidates(start, destination, route, robot, scale, line_width_mm=20, edge_validator=None, collision_checker=None, beam_width=4, min_track_mm=40):
    """Build both edge sides per leg; retain a bounded beam of feasible prefixes.
    Pose acquisition and corner transitions are nominal dead reckoning.
    """
    from sensor_geometry import generate_tracking_poses, generate_line_acquisition
    if beam_width < 1: raise ValueError('beam_width must be positive')
    beam = [((), start, 0.0)]
    for edge, reversed_edge in route:
        points = edge.points[::-1] if reversed_edge else edge.points
        a, b = points[0], points[-1]
        chord = math.dist(a,b)/scale
        if chord <= 1e-6 or edge.length_mm > chord*1.15:
            return []
        if max(math.dist(p,closest_point_on_segment(p,a,b)) for p in points)/scale > 5:
            return []
        legs = generate_tracking_poses(a,b,robot,scale,line_width_mm)
        legs = [leg for leg in legs if edge_validator is None or edge_validator(leg)]
        # Very short graph edges often occur near corners. Skip tracking on them;
        # the next leg's approach bridges the gap using direct movements.
        if not legs:
            continue
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
