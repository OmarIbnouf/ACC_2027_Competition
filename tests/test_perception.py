import unittest
from types import SimpleNamespace

from perception import (
    extract_quanser_detection_metadata,
    extract_sign_detection_metadata,
    normalize_class_name,
)


class PerceptionTests(unittest.TestCase):
    def test_normalizes_class_name_separators(self):
        self.assertEqual(normalize_class_name(" Traffic-Light "), "traffic light")

    def test_quanser_traffic_light_metadata_includes_color(self):
        detection = SimpleNamespace(
            name="Traffic Light", x=10, y=20, width=30, height=40, lightColor="Red"
        )

        self.assertEqual(
            extract_quanser_detection_metadata([detection]),
            [
                {
                    "class": "traffic light (red )",
                    "x": 10.0,
                    "y": 20.0,
                    "width": 30.0,
                    "height": 40.0,
                }
            ],
        )

    def test_quanser_missing_results_returns_empty_list(self):
        self.assertEqual(extract_quanser_detection_metadata(None), [])

    def test_sign_filter_keeps_only_stop_and_yield_signs(self):
        result = SimpleNamespace(
            boxes=SimpleNamespace(
                xywh=_Tensor(
                    [[10, 20, 30, 40], [50, 60, 70, 80], [90, 100, 110, 120]]
                ),
                cls=_Tensor([0, 1, 2]),
            ),
            names={0: "stop_sign", 1: "yield-sign", 2: "car"},
        )

        detections = extract_sign_detection_metadata(result)

        self.assertEqual(
            [detection["class"] for detection in detections],
            ["stop_sign", "yield-sign"],
        )


class _Tensor:
    def __init__(self, values):
        self.values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return _Array(self.values)


class _Array(list):
    def astype(self, value_type):
        return _Array(value_type(value) for value in self)


if __name__ == "__main__":
    unittest.main()
