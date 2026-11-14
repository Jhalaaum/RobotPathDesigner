import numpy as np
import math

def closest_point_on_segment(point, start, end):
    point = np.array(point)
    start = np.array(start)
    end = np.array(end)
    vector_start_point = point - start
    if np.array_equal(start, end):
        return start
    vector_start_end = end - start

    t = np.dot(vector_start_end, vector_start_point)/np.dot(vector_start_end, vector_start_end)
    t = max(0, min(t, 1))
    closest_point = start + t*(vector_start_end)
    return closest_point
def closest_point_on_graph(point, graph):
    closest_point_on_graph = None
    distance_to_closest_point = float("inf")
    closest_edge = None
    closest_segment_index = None
    for edge in graph.edges:
        for i in range(len(edge.points) - 1):
            start = edge.points[i]
            end = edge.points[i + 1]

            candidate = closest_point_on_segment(point, start, end)
            candidate_distance = math.dist(point, candidate)
            if distance_to_closest_point > candidate_distance:
                closest_point_on_graph = candidate
                distance_to_closest_point = candidate_distance
                closest_edge = edge
                closest_segment_index = i
    return closest_point_on_graph, closest_edge, closest_segment_index, distance_to_closest_point

def split_edge_points(edge, point, segment_index):
    new_edge_1 = edge.points[:segment_index+1]
    new_edge_2 = edge.points[segment_index+1:]
    point = tuple(point)
    if not np.allclose(new_edge_1[-1], point):
        new_edge_1 = new_edge_1 + (point,)
    if not np.allclose(new_edge_2[0], point):
        new_edge_2 = (point,) + new_edge_2
    return new_edge_1, new_edge_2
