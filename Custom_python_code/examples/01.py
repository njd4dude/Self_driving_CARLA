from pickle import FALSE
import carla
import random

## Part 1

# Connect to Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

settings = world.get_settings()
# settings.synchronous_mode = FALSE # Enables synchronous mode
# settings.fixed_delta_seconds = 0.01
world.apply_settings(settings)

# Get a vehicle from the library
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# Get a spawn point
spawn_points = world.get_map().get_spawn_points()

# Spawn a vehicle
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Autopilot
vehicle.set_autopilot(True)

# Get world spectator
spectator = world.get_spectator()

# Without the loop, the spectator won't follow the vehicle.
# wait_for_tick() sleeps until the server finishes a frame, then we move the camera.
while True:
    try:
        world.wait_for_tick()
        transform = carla.Transform(
            vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
            vehicle.get_transform().rotation,
        )
        spectator.set_transform(transform)

    except KeyboardInterrupt as e: 

        vehicle.destroy()

        print('Vehicles Destroyed. Bye!')
        break
