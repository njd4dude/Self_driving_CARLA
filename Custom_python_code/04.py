import carla

import random
import queue

import cv2
import numpy as np

## Part 1

# Connect to Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# Get a vehicle from the library
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# Get a spawn point
spawn_points = world.get_map().get_spawn_points()

# Spawn a vehicle
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
if vehicle is None:
    raise RuntimeError('Failed to spawn vehicle. Stop other scripts and try again.')

# Autopilot
vehicle.set_autopilot(True)

# Get the world spectator
spectator = world.get_spectator()

## Part 2

# Print all camera types
for bp in bp_lib.filter("camera"):
    print(bp.id)

# Create a camera floating behind the vehicle
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))

# Smaller images: six full-res extra cameras was crashing Unreal (Fatal Error).
IMAGE_W = '400'
IMAGE_H = '300'
image_w = int(IMAGE_W)
image_h = int(IMAGE_H)

def spawn_camera(blueprint_id):
    bp = bp_lib.find(blueprint_id)
    bp.set_attribute('image_size_x', IMAGE_W)
    bp.set_attribute('image_size_y', IMAGE_H)
    camera = world.spawn_actor(bp, camera_init_trans, attach_to=vehicle)
    print(f'spawned {blueprint_id}', flush=True)
    return camera

# DVS + optical flow use extra GPU paths and were crashing this machine.
# RGB / semantic / instance / depth still show the idea of multiple sensor types.
rgb_camera = spawn_camera('sensor.camera.rgb')
seg_camera = spawn_camera('sensor.camera.semantic_segmentation')
ins_camera = spawn_camera('sensor.camera.instance_segmentation')
depth_camera = spawn_camera('sensor.camera.depth')

def put_bgra(image, image_queue):
    frame = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    try:
        image_queue.put_nowait(frame)
    except queue.Full:
        try:
            image_queue.get_nowait()
        except queue.Empty:
            pass
        image_queue.put_nowait(frame)

def rgb_camera_callback(image, rgb_image_queue):
    put_bgra(image, rgb_image_queue)

def seg_camera_callback(image, seg_image_queue):
    image.convert(carla.ColorConverter.CityScapesPalette)
    put_bgra(image, seg_image_queue)

def ins_camera_callback(image, ins_image_queue):
    put_bgra(image, ins_image_queue)

def depth_camera_callback(image, depth_image_queue):
    image.convert(carla.ColorConverter.LogarithmicDepth)
    put_bgra(image, depth_image_queue)

rgb_image_queue = queue.Queue(maxsize=2)
seg_image_queue = queue.Queue(maxsize=2)
ins_image_queue = queue.Queue(maxsize=2)
depth_image_queue = queue.Queue(maxsize=2)

def latest_frame(image_queue, fallback):
    frame = fallback
    try:
        frame = image_queue.get(timeout=0.02)
        while True:
            frame = image_queue.get_nowait()
    except queue.Empty:
        pass
    return frame

def to_bgr(image):
    if image is None:
        return np.zeros((image_h, image_w, 3), dtype=np.uint8)
    return image[:, :, :3]

rgb_camera.listen(lambda image: rgb_camera_callback(image, rgb_image_queue))
seg_camera.listen(lambda image: seg_camera_callback(image, seg_image_queue))
ins_camera.listen(lambda image: ins_camera_callback(image, ins_image_queue))
depth_camera.listen(lambda image: depth_camera_callback(image, depth_image_queue))

cv2.namedWindow('All Cameras', cv2.WINDOW_NORMAL)
cv2.resizeWindow('All Cameras', 1280, 720)

def clear():
    for camera in (rgb_camera, seg_camera, ins_camera, depth_camera):
        camera.stop()
        camera.destroy()
    print('\nCameras Stopped.')
    vehicle.destroy()
    print('Vehicle Destroyed. Bye!')
    cv2.destroyAllWindows()

rgb_frame = seg_frame = ins_frame = depth_frame = None
while True:
    try:
        rgb_frame = latest_frame(rgb_image_queue, rgb_frame)
        seg_frame = latest_frame(seg_image_queue, seg_frame)
        ins_frame = latest_frame(ins_image_queue, ins_frame)
        depth_frame = latest_frame(depth_image_queue, depth_frame)

        top_row = np.hstack((to_bgr(rgb_frame), to_bgr(seg_frame)))
        lower_row = np.hstack((to_bgr(ins_frame), to_bgr(depth_frame)))
        tiled = np.vstack((top_row, lower_row))
        display = cv2.resize(tiled, (1280, 720))

        cv2.imshow('All Cameras', display)

        transform = carla.Transform(
            vehicle.get_transform().transform(carla.Location(x=-4, z=50)),
            carla.Rotation(yaw=-180, pitch=-90),
        )
        spectator.set_transform(transform)

        if cv2.waitKey(16) == ord('q'):
            clear()
            break

    except KeyboardInterrupt:
        clear()
        break
