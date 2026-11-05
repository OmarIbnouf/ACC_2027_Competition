import math
import unittest

from mission_planning import NODE_COORDS
from mission_runtime import get_optimal_path, order_stops
from traffic_control import detect_traffic_conditions


class TrafficControlTests(unittest.TestCase):
    def test_detects_signs_lights_cones_and_cars(self):
        detections = [
            {"class": "stop sign", "x": 300, "y": 200, "width": 60, "height": 70},
            {"class": "yield sign", "x": 300, "y": 200, "width": 50, "height": 70},
            {"class": "traffic light (green )", "x": 300, "y": 200, "width": 20, "height": 40},
            {"class": "traffic light (red )", "x": 300, "y": 200, "width": 20, "height": 40},
            {"class": "cone", "x": 320, "y": 300, "width": 40, "height": 50},
            {"class": "car", "x": 300, "y": 200, "width": 100, "height": 80},
        ]

        result = detect_traffic_conditions(detections, 10.0, 0.0)

        self.assertEqual(
            result,
            (True, True, False, True, True, True, 10.0),
        )

    def test_red_light_detection_respects_recent_green_light(self):
        red_light = {
            "class": "traffic light (red)",
            "x": 300,
            "y": 200,
            "width": 20,
            "height": 40,
        }

        result = detect_traffic_conditions([red_light], 10.0, 9.5)

        self.assertFalse(result[2])


class MissionRuntimeTests(unittest.TestCase):
    def test_get_optimal_path_uses_graph_fallback_when_no_roadmap_edge_matches(self):
        class Graph:
            def get_closest_edge(self, x, y):
                return 10, 1

            def find_shortest_path(self, start_node, end_node):
                return [start_node, end_node], 2.0

        class Roadmap:
            edges = []

        path, distance, target_node, target_edge_start = get_optimal_path(
            Graph(), Roadmap(), 5, [0.0, 0.0]
        )

        self.assertEqual(path, [5, 10, 1])
        self.assertAlmostEqual(
            distance, 2.0 + math.dist(NODE_COORDS[10], NODE_COORDS[1])
        )
        self.assertEqual((target_node, target_edge_start), (1, 10))

    def test_stop_order_uses_lowest_total_route_cost(self):
        pickup = {"name": "Pickup", "coords": 1}
        stop_a = {"name": "A", "coords": 2}
        stop_b = {"name": "B", "coords": 3}
        dropoff = {"name": "Drop-off", "coords": 4}
        hub = {"name": "Hub", "coords": 10}
        costs = {
            (10, 1): 1,
            (1, 2): 10,
            (1, 3): 1,
            (2, 3): 10,
            (3, 2): 1,
            (2, 4): 1,
            (3, 4): 10,
            (4, 10): 1,
        }

        def get_path(start, target):
            return [start, target], costs.get((start, target), 100), target, target

        ordered = order_stops([stop_a, stop_b], pickup, dropoff, hub, get_path)

        self.assertEqual([task["name"] for task in ordered], ["B", "A"])


if __name__ == "__main__":
    unittest.main()
