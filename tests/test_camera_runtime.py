import unittest
from types import SimpleNamespace

from camera_runtime import process_camera_models, render_camera_results


class EmptyBoxes:
    xywh = None
    xyxy = None

    def __len__(self):
        return 0


class FakeModel:
    def __init__(self, name, events, result):
        self.name = name
        self.events = events
        self.result = result

    def pre_process(self, image):
        self.events.append(f"{self.name}.pre_process")
        return image

    def predict(self, **kwargs):
        self.events.append(f"{self.name}.predict")
        return [self.result]

    def post_processing(self, **kwargs):
        self.events.append(f"{self.name}.post_processing")
        return []

    def post_process_render(self, **kwargs):
        self.events.append(f"{self.name}.render")
        return "rendered"


class CameraRuntimeTests(unittest.TestCase):
    def test_processes_depth_and_models_in_existing_order(self):
        events = []
        qcar_image = SimpleNamespace(
            rgb="camera image",
            depth="depth image",
            read=lambda: events.append("depth.read"),
        )
        empty_result = SimpleNamespace(boxes=EmptyBoxes(), names={})
        cone_model = FakeModel("cones", events, empty_result)
        segmentation_model = FakeModel("segmentation", events, None)

        detections, filtered_cones = process_camera_models(
            qcar_image, cone_model, segmentation_model
        )

        self.assertEqual(detections, [])
        self.assertIsNone(filtered_cones)
        self.assertEqual(
            events,
            [
                "depth.read",
                "cones.pre_process",
                "cones.predict",
                "cones.post_processing",
                "segmentation.pre_process",
                "segmentation.predict",
                "segmentation.post_processing",
            ],
        )
        self.assertEqual(
            render_camera_results(segmentation_model, filtered_cones), "rendered"
        )
        self.assertEqual(events[-1], "segmentation.render")


if __name__ == "__main__":
    unittest.main()
