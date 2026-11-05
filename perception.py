def normalize_class_name(name):
    return str(name).strip().lower().replace("_", " ").replace("-", " ")


def extract_detection_metadata(result):
    detections = []
    if result and getattr(result.boxes, "xywh", None) is not None:
        names = getattr(result, "names", {})
        for box, class_id in zip(
            result.boxes.xywh.detach().cpu().numpy(),
            result.boxes.cls.detach().cpu().numpy().astype(int),
        ):
            detections.append(
                {
                    "class": names.get(class_id, str(class_id)),
                    "x": float(box[0]),
                    "y": float(box[1]),
                    "width": float(box[2]),
                    "height": float(box[3]),
                }
            )
    return detections


def extract_quanser_detection_metadata(results):
    detections = []
    if results is None:
        return detections

    for obj in results:
        name = str(getattr(obj, "name", "")).strip()
        x = float(getattr(obj, "x", 0.0))
        y = float(getattr(obj, "y", 0.0))
        width = float(getattr(obj, "width", 0.0))
        height = float(getattr(obj, "height", 0.0))

        detection = {
            "class": name,
            "x": x,
            "y": y,
            "width": width,
            "height": height,
        }

        light_color = getattr(obj, "lightColor", None)
        if light_color is not None and "traffic light" in name.lower():
            color = str(light_color).strip().lower()
            if color:
                detection["class"] = f"traffic light ({color} )"

        detections.append(detection)

    return detections


def extract_sign_detection_metadata(result):
    return [
        det
        for det in extract_detection_metadata(result)
        if normalize_class_name(det["class"]) in ("stop sign", "yield sign")
    ]


def is_orange(image, box_xyxy):
    import cv2
    import numpy as np

    x1, y1, x2, y2 = map(int, box_xyxy)
    h, w = image.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    roi = image[y1:y2, x1:x2]
    if roi.size == 0:
        return False

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([0, 130, 150])
    upper_orange = np.array([25, 255, 255])
    mask_orange = cv2.inRange(hsv_roi, lower_orange, upper_orange)

    lower_crimson = np.array([165, 130, 150])
    upper_crimson = np.array([179, 255, 255])
    mask_crimson = cv2.inRange(hsv_roi, lower_crimson, upper_crimson)

    combined_mask = cv2.bitwise_or(mask_orange, mask_crimson)
    target_pixels = cv2.countNonZero(combined_mask)
    total_pixels = roi.shape[0] * roi.shape[1]
    return total_pixels > 0 and target_pixels / total_pixels > 0.15


def prepare_lane_threshold_image(frame, threshold_value):
    import cv2

    height = frame.shape[0]
    lower_half_frame = frame[int(height / 2):, :]
    gray = cv2.cvtColor(lower_half_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary_image = cv2.threshold(
        blurred, threshold_value, 255, cv2.THRESH_BINARY
    )
    return binary_image
