from perception import normalize_class_name
from app_config import (
    CONE_CENTER_THIRD_MAX_X,
    CONE_CENTER_THIRD_MIN_X,
    CONE_MIN_HEIGHT,
    YIELD_SIGN_MIN_WIDTH,
)


def detect_traffic_conditions(detections, current_time, green_light_last_seen_time):
    stop_sign_visible, yield_sign_visible = False, False
    red_or_yellow_light_visible, green_light_visible = False, False
    cone_detected_center = False
    car_in_front = False

    for obj in detections:
        obj_class = normalize_class_name(obj["class"])
        if obj_class in ("stop sign", "stopsign") and (
            obj["width"] > YIELD_SIGN_MIN_WIDTH
            or (obj["width"] < 15 and obj["y"] < 150 and obj["x"] > 550)
        ):
            stop_sign_visible = True
        elif obj_class in ("yield sign", "yieldsign") and (
            obj["width"] > YIELD_SIGN_MIN_WIDTH
            or (obj["width"] < 30 and obj["y"] < 150 and obj["x"] > 550)
        ):
            yield_sign_visible = True
        elif (
            "cone" in obj_class
            and CONE_CENTER_THIRD_MIN_X < obj["x"] < CONE_CENTER_THIRD_MAX_X
            and obj["y"] > 240
            and obj["height"] > CONE_MIN_HEIGHT
        ):
            cone_detected_center = True
        elif (
            obj_class in ("traffic light (green )", "traffic light (green)")
            and 20 < obj["x"] < 620
            and obj["y"] < 400
        ):
            green_light_visible = True
            green_light_last_seen_time = current_time
        elif (
            obj_class
            in (
                "traffic light (red )",
                "traffic light (red)",
                "traffic light (yellow )",
                "traffic light (yellow)",
            )
            and not current_time - green_light_last_seen_time < 1.0
            and 20 < obj["x"] < 620
            and obj["y"] < 400
        ):
            red_or_yellow_light_visible = True
        elif obj_class == "car":
            if 220 < obj["x"] < 420 and obj["width"] > 80:
                car_in_front = True
            elif obj["width"] > 250:
                car_in_front = True

    return (
        stop_sign_visible,
        yield_sign_visible,
        red_or_yellow_light_visible,
        green_light_visible,
        cone_detected_center,
        car_in_front,
        green_light_last_seen_time,
    )
