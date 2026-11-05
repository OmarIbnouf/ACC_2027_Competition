import math
import unittest

import numpy as np

from steering import SteeringController
from throttle import SpeedController


class SpeedControllerTests(unittest.TestCase):
    def test_pi_output_includes_proportional_and_integral_terms(self):
        controller = SpeedController(kp=0.1, ki=0.2)

        self.assertAlmostEqual(controller.update(1.0, 2.0, 0.5), 0.2)
        self.assertAlmostEqual(controller.ei, 0.5)

    def test_output_is_clamped_to_max_throttle(self):
        controller = SpeedController(kp=10)

        self.assertEqual(controller.update(0, 1, 1), controller.maxThrottle)
        self.assertEqual(controller.update(1, 0, 1), -controller.maxThrottle)


class SteeringControllerTests(unittest.TestCase):
    def test_tracks_straight_path_without_steering_error(self):
        controller = SteeringController(
            np.array([[0, 1, 2], [0, 0, 0]])
        )

        self.assertAlmostEqual(controller.update(np.array([0.5, 0]), 0, 1), 0)

    def test_corrects_toward_path_and_respects_steering_limit(self):
        controller = SteeringController(
            np.array([[0, 1, 2], [0, 0, 0]])
        )

        steering = controller.update(np.array([0.5, 1]), 0, 1)

        self.assertLess(steering, 0)
        self.assertGreaterEqual(steering, -math.pi / 6)

    def test_advances_waypoint_index_after_passing_segment(self):
        controller = SteeringController(
            np.array([[0, 1, 2], [0, 0, 0]]), cyclic=False
        )

        controller.update(np.array([1.2, 0]), 0, 1)

        self.assertEqual(controller.wpi, 1)


if __name__ == "__main__":
    unittest.main()
