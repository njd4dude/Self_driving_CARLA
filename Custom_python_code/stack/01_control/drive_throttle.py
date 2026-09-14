import carla
import random

client = carla.Client('localhost', 2000)
world = client.get_world()

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
spawn_points = world.get_map().get_spawn_points()
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# 1. Get the current physics control of the vehicle
physics_control = vehicle.get_physics_control()

# 2. Increase the maximum RPM to allow higher top speeds/longer acceleration
physics_control.max_rpm = 7000.0
# drop the mass of the vehicle to 300.0 kg
physics_control.mass = 100.0 

# 3. Create a higher torque curve 
# CARLA uses a list of 2D vectors: [RPM, Torque in Nm]
super_rocket_curve = [
    carla.Vector2D(0.0, 2000.0),    # Changed from 800 -> 2000 (Insane launch power)
    carla.Vector2D(2000.0, 3000.0),  # Changed from 1000 -> 3000 
    carla.Vector2D(5000.0, 3000.0),  # Changed from 1000 -> 3000
    carla.Vector2D(7000.0, 1500.0)   # Changed from 600 -> 1500
]
physics_control.torque_curve = super_rocket_curve

# 4. Apply the modified physics back to the vehicle
vehicle.apply_physics_control(physics_control)


spectator = world.get_spectator()

while True:
    try:
        world.wait_for_tick()
        vehicle.apply_control(carla.VehicleControl(throttle=1.0))

        transform = carla.Transform(
            vehicle.get_transform().transform(carla.Location(x=-8, z=3)),
            vehicle.get_transform().rotation,
        )
        spectator.set_transform(transform)

    except KeyboardInterrupt:
        vehicle.destroy()
        print('Vehicle destroyed. Bye!')
        break
