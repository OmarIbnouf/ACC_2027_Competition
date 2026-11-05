import math
import itertools

from mission_planning import NODE_COORDS


def get_optimal_path(city_graph, roadmap, start_node, target_coords):
    import numpy as np

    target_np = np.array(target_coords[:2], dtype=float)
    best_edge, best_dist = None, float("inf")
    for edge in roadmap.edges:
        if getattr(edge, "waypoints", None) is not None and edge.waypoints.size > 0:
            dists = np.hypot(
                edge.waypoints[0, :] - target_np[0],
                edge.waypoints[1, :] - target_np[1],
            )
            if np.min(dists) < best_dist:
                best_dist, best_edge = np.min(dists), edge

    if best_edge:
        u, v = roadmap.nodes.index(best_edge.fromNode), roadmap.nodes.index(
            best_edge.toNode
        )
    else:
        u, v = city_graph.get_closest_edge(target_coords[0], target_coords[1])

    path, path_dist = city_graph.find_shortest_path(start_node, u)

    if path is not None:
        path.append(v)
        p1, p2 = NODE_COORDS.get(u), NODE_COORDS.get(v)
        if p1 and p2:
            path_dist += math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    return path, path_dist, v, u


def order_stops(stop_tasks, pickup_dest, dropoff_dest, hub_dest, get_path):
    best_stops_order = []
    best_cost = float("inf")

    for perm in itertools.permutations(stop_tasks) if stop_tasks else [()]:
        current_start_node = 10
        current_cost = 0
        valid_route = True

        sequence_to_test = [pickup_dest] + list(perm) + [dropoff_dest, hub_dest]

        for task in sequence_to_test:
            path, path_dist, _, target_u = get_path(
                current_start_node, task["coords"]
            )
            if path is None:
                valid_route = False
                break

            current_cost += path_dist
            current_start_node = target_u

        if not valid_route:
            continue

        if current_cost < best_cost:
            best_cost = current_cost
            best_stops_order = list(perm)

    return best_stops_order
