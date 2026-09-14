"""Camera lane keeping: OpenCV lines + PID. Map offset is score-only.

CARLA must already be running.

    python stack/02_lane_keeping/lane_keep.py
"""

import math
import queue
import random
import sys

import carla
import cv2
import numpy as np

IMAGE_W = 640
IMAGE_H = 360
TARGET_SPEED = 8.0  # m/s ~ 18 mph; slow enough for Hough to keep up
MAX_THROTTLE = 0.45
KP = 0.9
KD = 0.25
KH = 0.35  # heading term from how the lane center shifts up the image


def speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def pick_spawn(world):
    carla_map = world.get_map()
    points = list(carla_map.get_spawn_points())
    random.shuffle(points)
    for sp in points:
        wp = carla_map.get_waypoint(sp.location)
        if wp is not None and not wp.is_junction:
            return sp
    return random.choice(points)


def map_lateral_offset_m(vehicle, carla_map):
    """Meters to the right of the lane center. Score only — not used to steer."""
    loc = vehicle.get_transform().location
    wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return None
    dx = loc.x - wp.transform.location.x
    dy = loc.y - wp.transform.location.y
    fwd = wp.transform.get_forward_vector()
    right_x, right_y = -fwd.y, fwd.x
    return dx * right_x + dy * right_y


def lane_mask(bgr):
    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    white = cv2.inRange(hls, np.array([0, 200, 0]), np.array([180, 255, 255]))
    yellow = cv2.inRange(hls, np.array([15, 30, 80]), np.array([40, 200, 255]))
    mask = cv2.bitwise_or(white, yellow)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def roi_mask(h, w):
    poly = np.array(
        [
            [
                (int(0.08 * w), h),
                (int(0.40 * w), int(0.58 * h)),
                (int(0.60 * w), int(0.58 * h)),
                (int(0.92 * w), h),
            ]
        ],
        dtype=np.int32,
    )
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, poly, 255)
    return mask, poly


def fit_x_of_y(lines):
    if not lines:
        return None
    xs, ys = [], []
    for x1, y1, x2, y2 in lines:
        xs.extend([x1, x2])
        ys.extend([y1, y2])
    if len(ys) < 2:
        return None
    m, b = np.polyfit(ys, xs, 1)
    return lambda y: m * y + b


def detect_lanes(bgr):
    h, w = bgr.shape[:2]
    mask = lane_mask(bgr)
    region, poly = roi_mask(h, w)
    masked = cv2.bitwise_and(mask, region)
    edges = cv2.Canny(masked, 50, 150)
    segs = cv2.HoughLinesP(
        edges, rho=1, theta=np.pi / 180, threshold=20, minLineLength=25, maxLineGap=80
    )

    left, right = [], []
    if segs is not None:
        for x1, y1, x2, y2 in segs.reshape(-1, 4):
            if x2 == x1:
                continue
            slope = (y2 - y1) / float(x2 - x1)
            if abs(slope) < 0.35 or abs(slope) > 5.0:
                continue
            seg = (x1, y1, x2, y2)
            if slope < 0:
                left.append(seg)
            else:
                right.append(seg)

    y_near = h - 2
    y_far = int(0.62 * h)
    x_left_near = x_right_near = x_left_far = x_right_far = None
    f_left = fit_x_of_y(left)
    f_right = fit_x_of_y(right)
    if f_left is not None:
        x_left_near = f_left(y_near)
        x_left_far = f_left(y_far)
    if f_right is not None:
        x_right_near = f_right(y_near)
        x_right_far = f_right(y_far)

    overlay = bgr.copy()
    cv2.polylines(overlay, poly, True, (80, 80, 80), 1)
    if f_left is not None:
        cv2.line(
            overlay,
            (int(f_left(y_near)), y_near),
            (int(f_left(y_far)), y_far),
            (255, 80, 80),
            4,
        )
    if f_right is not None:
        cv2.line(
            overlay,
            (int(f_right(y_near)), y_near),
            (int(f_right(y_far)), y_far),
            (80, 80, 255),
            4,
        )
    return overlay, (x_left_near, x_right_near, x_left_far, x_right_far)


def vision_errors(xs, width):
    """Normalized offset (right of image center is +) and heading proxy.

    Positive offset means the lane center is right of the camera, so the car
    is left of the lane and should steer right (CARLA steer > 0).
    """
    x_left_near, x_right_near, x_left_far, x_right_far = xs
    mid_near = mid_far = None
    if x_left_near is not None and x_right_near is not None:
        mid_near = 0.5 * (x_left_near + x_right_near)
    elif x_left_near is not None:
        mid_near = x_left_near + 0.25 * width
    elif x_right_near is not None:
        mid_near = x_right_near - 0.25 * width

    if x_left_far is not None and x_right_far is not None:
        mid_far = 0.5 * (x_left_far + x_right_far)
    elif x_left_far is not None:
        mid_far = x_left_far + 0.25 * width
    elif x_right_far is not None:
        mid_far = x_right_far - 0.25 * width

    if mid_near is None:
        return None, None, None

    offset = (mid_near - width / 2.0) / (width / 2.0)
    heading = 0.0
    if mid_far is not None:
        heading = (mid_far - mid_near) / (width / 2.0)
    return offset, heading, mid_near


def follow_spectator(spectator, vehicle):
    transform = carla.Transform(
        vehicle.get_transform().transform(carla.Location(x=-8, z=3)),
        vehicle.get_transform().rotation,
    )
    spectator.set_transform(transform)


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle = world.try_spawn_actor(vehicle_bp, pick_spawn(world))
    if vehicle is None:
        world.apply_settings(original)
        raise RuntimeError('Failed to spawn. Stop other scripts and try again.')

    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(IMAGE_W))
    cam_bp.set_attribute('image_size_y', str(IMAGE_H))
    cam_bp.set_attribute('fov', '90')
    camera = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(x=1.6, z=1.4), carla.Rotation(pitch=-8)),
        attach_to=vehicle,
    )
    frames = queue.Queue(maxsize=2)
    camera.listen(frames.put)

    spectator = world.get_spectator()
    carla_map = world.get_map()
    prev_offset = 0.0
    steer = 0.0
    tick = 0

    def shutdown():
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()
        print('Cleaned up. Bye!')

    print('Lane keep running. Focus the OpenCV window, q or Ctrl+C to quit.')
    try:
        while True:
            world.tick()
            image = frames.get(timeout=2.0)
            bgra = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
            bgr = bgra[:, :, :3]

            overlay, xs = detect_lanes(bgr)
            offset, heading, mid_near = vision_errors(xs, IMAGE_W)

            if offset is None:
                throttle = 0.15
                steer *= 0.9
            else:
                deriv = offset - prev_offset
                prev_offset = offset
                steer = KP * offset + KD * deriv + KH * heading
                steer = float(np.clip(steer, -1.0, 1.0))
                speed = speed_mps(vehicle)
                throttle = float(np.clip(0.12 * (TARGET_SPEED - speed), 0.0, MAX_THROTTLE))
                if mid_near is not None:
                    cv2.circle(overlay, (int(mid_near), IMAGE_H - 2), 6, (0, 255, 255), -1)

            vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
            follow_spectator(spectator, vehicle)

            truth = map_lateral_offset_m(vehicle, carla_map)
            hud = 'steer %.2f  speed %.1f  vision_off %.2f  map_off_m %s' % (
                steer,
                speed_mps(vehicle),
                0.0 if offset is None else offset,
                'n/a' if truth is None else '%.2f' % truth,
            )
            cv2.putText(overlay, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(overlay, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.line(overlay, (IMAGE_W // 2, 0), (IMAGE_W // 2, IMAGE_H), (0, 255, 0), 1)
            cv2.imshow('lane_keep (camera only)', overlay)
            if cv2.waitKey(1) == ord('q'):
                break

            tick += 1
            if tick % 20 == 0:
                print(hud)

    except (KeyboardInterrupt, queue.Empty):
        pass
    finally:
        shutdown()


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
