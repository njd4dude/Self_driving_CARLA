import carla
import random

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

import carla

settings = world.get_settings()

print("Synchronous mode:", settings.synchronous_mode)
print("Fixed delta seconds:", settings.fixed_delta_seconds)