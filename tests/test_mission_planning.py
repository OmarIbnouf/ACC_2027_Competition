import math
import unittest

from mission_planning import (
    NODE_COORDS,
    PREDEFINED_RIDES,
    CityGraph,
    is_point_in_crosswalk,
)


class CityGraphTests(unittest.TestCase):
    def setUp(self):
        self.graph = CityGraph()

    def test_finds_shortest_path_using_physical_distance(self):
        path, distance = self.graph.find_shortest_path(10, 8)

        self.assertEqual(path, [10, 1, 8])
        expected_distance = math.dist(NODE_COORDS[10], NODE_COORDS[1]) + math.dist(
            NODE_COORDS[1], NODE_COORDS[8]
        )
        self.assertAlmostEqual(distance, expected_distance)

    def test_same_start_and_end_returns_single_node(self):
        self.assertEqual(self.graph.find_shortest_path(10, 10), ([10], 0.0))

    def test_all_predefined_ride_nodes_have_coordinates(self):
        for ride_nodes in PREDEFINED_RIDES.values():
            for node in ride_nodes:
                with self.subTest(node=node):
                    self.assertIn(node, NODE_COORDS)


class CrosswalkTests(unittest.TestCase):
    def test_crosswalk_rectangle_includes_its_boundary(self):
        self.assertTrue(is_point_in_crosswalk((-2.067, 0.070)))

    def test_point_outside_crosswalks_is_rejected(self):
        self.assertFalse(is_point_in_crosswalk((10.0, 10.0)))


if __name__ == "__main__":
    unittest.main()
