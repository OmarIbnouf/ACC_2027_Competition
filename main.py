# main.py - Updated with Corrected Hardware Timing & EKF Tracking
#file changed
import os
import signal
import threading
import time
from threading import Lock, Thread

import cv2
import numpy as np
import pyqtgraph as pg

# --- QCar Control & Visualization Imports ---
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.vision import Camera2D
from hal.content.qcar_functions import QCarEKF
from pal.utilities.scope import MultiScope
import pal.resources.images as images

try:
    from pit.YOLO.nets import YOLOv8
    from pit.YOLO.utils import QCar2DepthAligned
except ImportError:
    print(
        "Warning: YOLO dependencies not found. Visualization may fail if not simulated."
    )

from custom_roadmap import CustomRoadMap
from app_config import (
    CAMERA_ID,
    CONE_AVOID_COOLDOWN_S,
    CONE_AVOID_LEFT_STEER,
    CONE_AVOID_PHASE_DURATION_S,
    CONE_AVOID_RIGHT_STEER,
    CONE_AVOID_TARGET_SPEED,
    CONTROLLER_UPDATE_RATE,
    ENABLE_CONE_AVOIDANCE as DEFAULT_ENABLE_CONE_AVOIDANCE,
    ENABLE_STEERING_CONTROL as enableSteeringControl,
    FRAME_RATE,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    K_I,
    K_P,
    K_STANLEY as K_stanley,
    SIGN_REARM_TIME_S,
    STARTING_ORIENTATION_DEG,
    STARTING_POSITION,
    STOP_SIGN_WAIT_TIME_S,
    TAXI_HUB_POS,
    V_REF,
    YIELD_SIGN_WAIT_TIME_S,
)
from camera_runtime import process_camera_models, render_camera_results
from hardware_adapters import PhysicalLEDs
from mission_runtime import get_optimal_path, order_stops
from mission_planning import (
    NODE_COORDS,
    PREDEFINED_RIDES,
    CityGraph,
)
from perception import prepare_lane_threshold_image
from steering import SteeringController
from throttle import SpeedController
from traffic_control import detect_traffic_conditions

os.environ["QT_LOGGING_RULES"] = "*.debug=false;*.warning=false;qt.*=false"
v_ref = V_REF
K_p = K_P
K_i = K_I
ENABLE_CONE_AVOIDANCE = DEFAULT_ENABLE_CONE_AVOIDANCE

input_data = {"pickup": None, "dropoff": None, "stops": [], "ready": False}
ROADMAP_WAYPOINTS = None
KILL_THREAD = False
latest_detections = []
detections_lock = Lock()
frame_lock = threading.Lock()
latest_frame = None


# ============================
# Robust Shutdown Handler
# ============================
def sig_handler(signum, frame):
    global KILL_THREAD
    if KILL_THREAD:
        print("\n[WARNING] Forced termination initiated.")
        os._exit(1)

    KILL_THREAD = True
    print(
        "\n[CTRL+C] Shutdown signal received. Cleaning up gracefully... (Press Ctrl+C again to force quit)"
    )
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, sig_handler)


def collect_roadmap_waypoints(roadmap):
    points = []
    for edge in roadmap.edges:
        if getattr(edge, "waypoints", None) is not None and edge.waypoints.size > 0:
            points.append(edge.waypoints[:2, :].T)
    return np.vstack(points) if points else np.empty((0, 2))


# ============================
# Input Thread Worker (Updated for Ride Selection)
# ============================
def input_thread_worker():
    try:
        print("\n" + "="*35)
        print(" ACC 2026 RIDE SELECTOR")
        print("="*35)
        
        # Display the routes nicely formatted
        routes_list = list(PREDEFINED_RIDES.items())
        half_len = len(routes_list) // 2
        for i in range(half_len):
            col1 = f" {routes_list[i][0]}: Route {routes_list[i][1]}"
            col2 = f" {routes_list[i + half_len][0]}: Route {routes_list[i + half_len][1]}"
            print(f"{col1:<20} | {col2}")
        print("="*35)
        
        while True:
            ride_choice = input("Enter the letter of the ride (A-X): ").strip().upper()
            if ride_choice in PREDEFINED_RIDES:
                break
            print("Invalid choice. Please enter a valid letter.")

        ride_nodes = PREDEFINED_RIDES[ride_choice]
        
        pickup_node = ride_nodes[0]
        dropoff_node = ride_nodes[-1]
        intermediate_nodes = ride_nodes[1:-1]

        input_data["pickup"] = list(NODE_COORDS[pickup_node])
        input_data["dropoff"] = list(NODE_COORDS[dropoff_node])
        input_data["stops"] = [list(NODE_COORDS[n]) for n in intermediate_nodes]
        input_data["ready"] = True

    except EOFError:
        pass


def wait_keep_alive(qcar, gps, ekf, duration):
    start_time = time.time()
    t0 = time.time()
    t = 0
    while time.time() - start_time < duration and not KILL_THREAD:
        tp = t
        t = time.time() - t0
        dt = t - tp
        if dt <= 0:
            dt = 0.001

        qcar.read()  # Blocks to maintain frequency
        v = float(qcar.motorTach)

        if gps.readGPS():
            ekf.update(
                [v, 0.0],
                dt,
                np.array([gps.position[0], gps.position[1], gps.orientation[2]]),
                qcar.gyroscope[2],
            )
        else:
            ekf.update([v, 0.0], dt, None, qcar.gyroscope[2])

        qcar.write(0, 0)


def execute_path(
    qcar,
    gps,
    ekf,
    speed_ctrl,
    path_waypoints,
    target_coord,
    roadmap,
    arrow_item,
    plot_time_offset,
    car_leds,
    base_color,
):
    path_points_2d = path_waypoints[:2, :].T
    closest_idx = int(
        np.argmin(np.linalg.norm(path_points_2d - np.array(target_coord), axis=1))
    )
    truncated_waypoints = path_waypoints[:, : closest_idx + 1]

    steer_ctrl = SteeringController(
        waypoints=truncated_waypoints, k=K_stanley, cyclic=False
    )

    qcar.read()
    current_pos = np.array([ekf.x_hat[0, 0], ekf.x_hat[1, 0]])
    search_limit = min(200, truncated_waypoints.shape[1])
    steer_ctrl.wpi = int(
        np.argmin(
            np.linalg.norm(
                truncated_waypoints[:2, :search_limit].T - current_pos, axis=1
            )
        )
    )

    # CRITICAL: Reset integral error before starting a new path segment
    speed_ctrl.ei = 0

    stop_sign_start_time, yield_sign_start_time = 0.0, 0.0
    stop_sign_last_seen_time, yield_sign_last_seen_time = 0.0, 0.0
    red_light_last_seen_time = 0.0
    green_light_last_seen_time = 0.0

    red_detection_start = 0.0
    is_stopping_for_light = False

    is_stopped_for_sign, is_stopped_for_yield = False, False
    cone_avoid_active, cone_avoid_start_time, cone_avoid_cooldown_until = (
        False,
        0.0,
        0.0,
    )
    count, countMax = 0, CONTROLLER_UPDATE_RATE / 10
    led_is_red = False

    t0 = time.time()
    t = 0
    str_ang = 0.0

    while not KILL_THREAD:
        tp = t
        t = time.time() - t0
        dt = t - tp
        if dt <= 0:
            dt = 0.001

        qcar.read()  # Blocks to maintain exactly 100Hz
        current_time = time.time()
        v = float(qcar.motorTach)

        # CRITICAL: Pass the actual steering angle (str_ang) into EKF
        if gps.readGPS():
            ekf.update(
                [v, str_ang],
                dt,
                np.array([gps.position[0], gps.position[1], gps.orientation[2]]),
                qcar.gyroscope[2],
            )
        else:
            ekf.update([v, str_ang], dt, None, qcar.gyroscope[2])

        x, y, th = (
            float(ekf.x_hat[0, 0]),
            float(ekf.x_hat[1, 0]),
            float(ekf.x_hat[2, 0]),
        )
        p_front = np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2

        with detections_lock:
            current_detections = list(latest_detections)

        (
            stop_sign_visible,
            yield_sign_visible,
            red_or_yellow_light_visible,
            green_light_visible,
            cone_detected_center,
            car_in_front,
            green_light_last_seen_time,
        ) = detect_traffic_conditions(
            current_detections, current_time, green_light_last_seen_time
        )

        if stop_sign_visible:
            stop_sign_last_seen_time = current_time
        if yield_sign_visible:
            yield_sign_last_seen_time = current_time
        should_stop = False

        if red_or_yellow_light_visible:
            if red_detection_start == 0.0:
                red_detection_start = current_time
            elif current_time - red_detection_start > 0.01:
                is_stopping_for_light = True
        else:
            red_detection_start = 0.0
            is_stopping_for_light = False

        if is_stopping_for_light:
            should_stop = True
        elif green_light_visible:
            should_stop = False

        if current_time - red_light_last_seen_time < 0.5:
            should_stop = True

        if current_time - green_light_last_seen_time < 2.0:
            should_stop = False

        if current_time - yield_sign_last_seen_time < SIGN_REARM_TIME_S:
            if not is_stopped_for_yield:
                should_stop = True
                if yield_sign_start_time == 0.0:
                    yield_sign_start_time = current_time
                elif current_time - yield_sign_start_time > YIELD_SIGN_WAIT_TIME_S:
                    is_stopped_for_yield = True
                    yield_sign_start_time = 0.0
        else:
            yield_sign_start_time, is_stopped_for_yield = 0.0, False

        if current_time - stop_sign_last_seen_time < SIGN_REARM_TIME_S:
            if not is_stopped_for_sign:
                should_stop = True
                if stop_sign_start_time == 0.0:
                    stop_sign_start_time = current_time
                elif current_time - stop_sign_start_time > STOP_SIGN_WAIT_TIME_S:
                    is_stopped_for_sign = True
                    stop_sign_start_time = 0.0
        else:
            stop_sign_start_time, is_stopped_for_sign = 0.0, False

        if car_in_front:
            should_stop = True

        # Gated by the new ENABLE_CONE_AVOIDANCE flag
        if (
            ENABLE_CONE_AVOIDANCE
            and cone_detected_center
            and not cone_avoid_active
            and current_time >= cone_avoid_cooldown_until
        ):
            cone_avoid_active, cone_avoid_start_time = True, current_time

        target_speed = v_ref
        if cone_avoid_active:
            should_stop = False
            target_speed = min(v_ref, CONE_AVOID_TARGET_SPEED)
            elapsed = current_time - cone_avoid_start_time
            if elapsed < CONE_AVOID_PHASE_DURATION_S:
                str_ang = CONE_AVOID_LEFT_STEER
            elif elapsed < 2.0 * CONE_AVOID_PHASE_DURATION_S:
                str_ang = CONE_AVOID_RIGHT_STEER
            else:
                cone_avoid_active, cone_avoid_cooldown_until = (
                    False,
                    current_time + CONE_AVOID_COOLDOWN_S,
                )

        if should_stop:
            qcar.write(0, 0)
            str_ang = 0.0
            speed_ctrl.ei = 0  # Prevent windup while stopped
        else:

            if not cone_avoid_active:
                str_ang = steer_ctrl.update(p_front, th, v)

            thr = speed_ctrl.update(v, target_speed, dt)
            qcar.write(thr, str_ang)

            if steer_ctrl.wpi >= steer_ctrl.N - 2:
                qcar.write(0, 0)
                break

        count += 1
        if count >= countMax:
            t_plot = current_time - plot_time_offset
            if arrow_item:
                arrow_item.setPos(x, y)
                arrow_item.setStyle(angle=180 - th * 180 / np.pi)
            if "steeringScope" in globals():
                steeringScope.axes[0].sample(t_plot, [[x, y]])
            count = 0


def mission_coordinator_loop(calibrate_gps=False):
    global KILL_THREAD, ROADMAP_WAYPOINTS
    qcar = QCar(readMode=1, frequency=CONTROLLER_UPDATE_RATE)
    ekf = QCarEKF(
        x_0=[
            STARTING_POSITION[0],
            STARTING_POSITION[1],
            np.deg2rad(STARTING_ORIENTATION_DEG[2]),
        ]
    )
    gps = QCarGPS(initialPose=[0, 2, -np.pi / 2], calibrate=calibrate_gps)

    speed_ctrl = SpeedController(kp=K_p, ki=K_i)
    city_graph = CityGraph()
    roadmap = CustomRoadMap()
    ROADMAP_WAYPOINTS = collect_roadmap_waypoints(roadmap)

    with qcar, gps:
        plot_time_offset = time.time()
        car_leds = PhysicalLEDs()

        while not KILL_THREAD:
            input_data["ready"] = False
            Thread(target=input_thread_worker, daemon=True).start()
            print(
                "Status: Waiting for Ride Selection (Car Idling at Hub)... [LED: Magenta]"
            )
            car_leds.set_color(255, 0, 255)

            while not input_data["ready"] and not KILL_THREAD:
                wait_keep_alive(qcar, gps, ekf, 0.5)

            if KILL_THREAD:
                break

            print("Planning routes...")

            pickup_dest = {
                "name": "Pickup",
                "coords": input_data["pickup"],
                "wait_time": 5,
                "led": (0, 0, 255),
                "wait_msg": "Passenger Pickup Wait (5s)",
            }
            dropoff_dest = {
                "name": "Drop-off",
                "coords": input_data["dropoff"],
                "wait_time": 5,
                "led": (255, 165, 0),
                "wait_msg": "Passenger Drop-off Wait (5s)",
            }
            hub_dest = {
                "name": "Hub",
                "coords": TAXI_HUB_POS[:2],
                "wait_time": 0,
                "led": (255, 0, 255),
                "wait_msg": "Mission Complete. Idling at Hub.",
            }

            stop_tasks = []
            for i, stop in enumerate(input_data["stops"]):
                stop_tasks.append(
                    {
                        "name": f"Stop {i+1}",
                        "coords": stop,
                        "wait_time": 3,
                        "led": (255, 0, 0),
                        "wait_msg": f"Intermediate Stop {i+1} Wait (3s)",
                    }
                )

            best_stops_order = order_stops(
                stop_tasks,
                pickup_dest,
                dropoff_dest,
                hub_dest,
                lambda start_node, coords: get_optimal_path(
                    city_graph, roadmap, start_node, coords
                ),
            )

            destinations = [pickup_dest] + best_stops_order + [dropoff_dest, hub_dest]

            route_names = [t["name"] for t in destinations]
            print(f"Optimal route found: Hub -> {' -> '.join(route_names)}")

            current_start_node = 10
            active_edge_u = None

            for dest in destinations:
                if KILL_THREAD:
                    break

                path, _, current_start_node, target_u = get_optimal_path(
                    city_graph, roadmap, current_start_node, dest["coords"]
                )

                if active_edge_u is not None and path and active_edge_u != path[0]:
                    path.insert(0, active_edge_u)

                active_edge_u = target_u

                wp = roadmap.generate_path(path)

                print(f"Status: Moving to {dest['name']} via {path} [LED: Green]")
                car_leds.set_color(0, 255, 0)
                if "referencePath" in globals() and wp is not None:
                    referencePath.setData(wp[0, :], wp[1, :])

                execute_path(
                    qcar,
                    gps,
                    ekf,
                    speed_ctrl,
                    wp,
                    dest["coords"],
                    roadmap,
                    arrow if "arrow" in globals() else None,
                    plot_time_offset,
                    car_leds,
                    (0, 255, 0),
                )

                print(f"Status: {dest['wait_msg']} [LED: {dest['led']}]")
                car_leds.set_color(*dest["led"])
                if dest["wait_time"] > 0:
                    wait_keep_alive(qcar, gps, ekf, dest["wait_time"])


# ============================
# Main Loop (Visualization & Detection)
# ============================
if __name__ == "__main__":

    if not IS_PHYSICAL_QCAR:
        import qlabs_setup

        qlabs_setup.setup(
            initialPosition=[STARTING_POSITION[0], STARTING_POSITION[1], 0],
            initialOrientation=[0, 0, np.deg2rad(STARTING_ORIENTATION_DEG[2])],
        )

    fps = 10 if IS_PHYSICAL_QCAR else 30
    print("Starting QCar hardware and GPS connection...")
    do_calibrate = "y" in input("Do you want to recalibrate the GPS? (y/n): ").lower()
    
    try:
        user_speed = input(
            f"Enter target speed in m/s (ex:0.5,0.65,0.75) [Enter for default {v_ref}]: "
        )
        if user_speed.strip() != "":
            v_ref = float(user_speed)
    except ValueError:
        print(f"Invalid input. Defaulting to {v_ref} m/s")

    # Ask the user if they want cone avoidance at script start
    cone_ans = input("Enable traffic cone avoidance? (y/n) [Enter for default 'n']: ").strip().lower()
    if cone_ans == 'y':
        ENABLE_CONE_AVOIDANCE = True
        print("Traffic cone avoidance ENABLED.")
    else:
        ENABLE_CONE_AVOIDANCE = False
        print("Traffic cone avoidance DISABLED.")

    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4, cols=2, title="Vehicle Steering Control", fps=fps
        )
        steeringScope.addXYAxis(
            row=0,
            col=1,
            rowSpan=4,
            xLabel="x Position [m]",
            yLabel="y Position [m]",
            xLim=(-2.5, 2.5),
            yLim=(-1, 5),
        )
        steeringScope.axes[0].plot.setAspectLocked(True)
        im = cv2.imread(images.SDCS_CITYSCAPE, cv2.IMREAD_GRAYSCALE)
        steeringScope.axes[0].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125, 2365),
            rotation=180,
            levels=(0, 255),
        )
        steeringScope.axes[0].images[0].setImage(image=im)

        referencePath = pg.PlotDataItem(
            pen={"color": (85, 168, 104), "width": 2}, name="Reference"
        )
        steeringScope.axes[0].plot.addItem(referencePath)
        steeringScope.axes[0].attachSignal(name="Estimated", width=2)

        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={"color": "w", "fillColor": [196, 78, 82], "width": 1},
            brush=[196, 78, 82],
        )
        arrow.setPos(STARTING_POSITION[0], STARTING_POSITION[1])
        steeringScope.axes[0].plot.addItem(arrow)

    print("Loading YOLO models and warming up TensorRT... please wait.")
    cone_yolo = YOLOv8(
        modelPath="traffic_cones.pt", imageHeight=IMAGE_HEIGHT, imageWidth=IMAGE_WIDTH
    )
    seg_yolo = YOLOv8(
        modelPath="yolov8s-seg.pt", imageHeight=IMAGE_HEIGHT, imageWidth=IMAGE_WIDTH
    )
    qcar_img = QCar2DepthAligned()
    camera = Camera2D(
        cameraId=CAMERA_ID,
        frameWidth=IMAGE_WIDTH,
        frameHeight=IMAGE_HEIGHT,
        frameRate=FRAME_RATE,
    )

    control_thread = Thread(
        target=mission_coordinator_loop, args=(do_calibrate,), daemon=True
    )
    control_thread.start()
    threshold_value = 115
    SAMPLE_TIME = 1.0 / 30.0

    print("Experiment started. Press Ctrl+C in terminal to stop.")
    try:
        while control_thread.is_alive() and (not KILL_THREAD):
            start = time.time()
            if camera.read():
                raw_frame = camera.imageData
                binaryImage = prepare_lane_threshold_image(raw_frame, threshold_value)

                with frame_lock:
                    latest_frame = binaryImage

            current_detections, filtered_res_cones = process_camera_models(
                qcar_img, cone_yolo, seg_yolo
            )

            with detections_lock:
                latest_detections.clear()
                latest_detections.extend(current_detections)

            combined_img = render_camera_results(seg_yolo, filtered_res_cones)

           # cv2.imshow("Object Segmentation", combined_img)
            MultiScope.refreshAll()

            end = time.time()
            sleep_time = max(SAMPLE_TIME - ((end - start) % SAMPLE_TIME), 1e-3)
            cv2.waitKey(max(int(1000 * sleep_time), 1))

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        print("\n[INFO] User interrupted! Shutting down...")
        KILL_THREAD = True

        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass

        if not IS_PHYSICAL_QCAR:
            try:
                import qlabs_setup

                qlabs_setup.terminate()
            except Exception:
                pass

        if control_thread.is_alive():
            control_thread.join(timeout=2)

        print("Experiment complete.")