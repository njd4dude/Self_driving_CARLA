"""Control: hold a target speed by capping throttle instead of flooring it.

CARLA must already be running.

    python stack/01_control/hold_speed.py
"""

import math

import carla
import random

TARGET_SPEED = 10.0  # m/s ~ 22 mph
KP = 0.35            # throttle/brake per m/s of error
MAX_THROTTLE = 0.75


def speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def speed_control(vehicle, target_speed):
    """Proportional throttle/brake to hold target_speed. No perception used."""
    error = target_speed - speed_mps(vehicle)
    if error >= 0:
        throttle = min(KP * error, MAX_THROTTLE)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(-KP * error, 1.0)
    return carla.VehicleControl(throttle=throttle, brake=brake)


client = carla.Client('localhost', 2000)
world = client.get_world()

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
spawn_points = world.get_map().get_spawn_points()
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

spectator = world.get_spectator()

tick = 0
try:
    while True:
        world.wait_for_tick()
        vehicle.apply_control(speed_control(vehicle, TARGET_SPEED))

        transform = carla.Transform(
            vehicle.get_transform().transform(carla.Location(x=-8, z=3)),
            vehicle.get_transform().rotation,
        )
        spectator.set_transform(transform)

        tick += 1
        if tick % 20 == 0:
            print('speed %.1f m/s (target %.1f)' % (speed_mps(vehicle), TARGET_SPEED))

except KeyboardInterrupt:
    vehicle.destroy()
    print('Vehicle destroyed. Bye!')
