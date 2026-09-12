"""Populate the current CARLA map with NPC vehicles, an ego vehicle, and pedestrians.

CarlaUE4.exe must already be running (use load_map.py first if you need a town).

The ego vehicle is the actor an autonomous agent would control. NPC vehicles use
Traffic Manager autopilot. Pedestrians walk to random navigation points.

An RGB camera is attached to the ego vehicle and frames are saved under _out/.

Press Ctrl+C or ESC to destroy spawned actors and exit.

Click the pygame window so keys go to the ego Mustang.

  W / S         throttle / brake
  A / D         steer
  Space         handbrake
  Q             reverse
  ESC           quit

Examples:
    python spawn_npcs.py
    python spawn_npcs.py --vehicles 50 --walkers 20
    python spawn_npcs.py --no-autopilot
    python spawn_npcs.py --follow
    python spawn_npcs.py --output _out
    python spawn_npcs.py --ego-autopilot
"""

import argparse
import os
import random
import time

import carla

try:
    import pygame
    from pygame.locals import K_ESCAPE, K_SPACE, K_a, K_d, K_q, K_s, K_w
except ImportError:
    raise RuntimeError('pygame is required. Install it with: pip install pygame')


def connect(host, port, timeout=10.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client, client.get_world()


def spawn_npc_vehicles(world, count):
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)

    vehicles = []
    attempts = min(count, len(spawn_points))
    for i in range(attempts):
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            blueprint.set_attribute(
                'color', random.choice(blueprint.get_attribute('color').recommended_values)
            )
        blueprint.set_attribute('role_name', 'autopilot')
        vehicle = world.try_spawn_actor(blueprint, spawn_points[i])
        if vehicle is not None:
            vehicles.append(vehicle)
    return vehicles, spawn_points


def spawn_ego_vehicle(world, spawn_points, used_count):
    library = world.get_blueprint_library()
    blueprint = library.find('vehicle.ford.mustang')
    leftover = spawn_points[used_count:] or spawn_points
    random.shuffle(leftover)

    if blueprint.has_attribute('color'):
        blueprint.set_attribute(
            'color', random.choice(blueprint.get_attribute('color').recommended_values)
        )
    blueprint.set_attribute('role_name', 'hero')

    for transform in leftover:
        ego = world.try_spawn_actor(blueprint, transform)
        if ego is not None:
            return ego
    raise RuntimeError('Could not spawn the ego vehicle (all spawn points occupied).')


def spawn_walkers(world, count):
    walker_blueprints = world.get_blueprint_library().filter('walker.pedestrian.*')
    controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    walkers = []
    controllers = []
    for _ in range(count):
        location = world.get_random_location_from_navigation()
        if location is None:
            continue
        walker_bp = random.choice(walker_blueprints)
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        walker = world.try_spawn_actor(walker_bp, carla.Transform(location))
        if walker is None:
            continue
        controller = world.spawn_actor(controller_bp, carla.Transform(), walker)
        walkers.append(walker)
        controllers.append(controller)

    world.wait_for_tick()
    for controller in controllers:
        controller.start()
        destination = world.get_random_location_from_navigation()
        if destination is not None:
            controller.go_to_location(destination)
        controller.set_max_speed(1.4)

    return walkers, controllers


def attach_rgb_camera(world, ego_vehicle, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    camera_init_trans = carla.Transform(carla.Location(z=1.5))
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
    pattern = os.path.join(output_dir, '%06d.png').replace('\\', '/')
    camera.listen(lambda image: image.save_to_disk(pattern % image.frame))
    return camera


def follow_ego(world, ego):
    spectator = world.get_spectator()
    transform = ego.get_transform()
    forward = transform.get_forward_vector()
    spectator.set_transform(
        carla.Transform(
            transform.location - 8.0 * forward + carla.Location(z=3.5),
            carla.Rotation(pitch=-12.0, yaw=transform.rotation.yaw),
        )
    )


DRIVE_HELP = [
    'Ego Mustang',
    '',
    'W/S  throttle/brake',
    'A/D  steer     Space  handbrake',
    'Q    reverse   ESC    quit',
]


def drive_ego(world, ego, follow):
    pygame.init()
    pygame.display.set_caption('CARLA ego — click this window to drive')
    screen = pygame.display.set_mode((420, 200))
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    reverse = False

    print('Click the pygame window, then use WASD to drive.')
    running = True
    while running:
        clock.tick(60)
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                elif event.key == K_q:
                    reverse = not reverse
                    print('Reverse %s' % ('on' if reverse else 'off'))

        control = carla.VehicleControl()
        control.throttle = 1.0 if keys[K_w] else 0.0
        control.brake = 1.0 if keys[K_s] else 0.0
        steer = 0.0
        if keys[K_a]:
            steer -= 0.7
        if keys[K_d]:
            steer += 0.7
        control.steer = steer
        control.hand_brake = bool(keys[K_SPACE])
        control.reverse = reverse
        ego.apply_control(control)

        world.wait_for_tick()
        if follow:
            follow_ego(world, ego)

        screen.fill((20, 20, 24))
        y = 16
        for line in DRIVE_HELP:
            color = (230, 230, 230) if line else (20, 20, 24)
            screen.blit(font.render(line, True, color), (16, y))
            y += 28
        pygame.display.flip()

    pygame.quit()


def destroy_actors(client, actors):
    actor_ids = []
    for actor in actors:
        if actor is None:
            continue
        try:
            actor_ids.append(actor.id)
        except Exception:
            pass
    if actor_ids:
        client.apply_batch([carla.command.DestroyActor(actor_id) for actor_id in actor_ids])


def traffic_actors(world):
    actors = world.get_actors()
    return (
        list(actors.filter('vehicle.*'))
        + list(actors.filter('walker.*'))
        + list(actors.filter('controller.ai.walker'))
        + list(actors.filter('sensor.*'))
    )


def clear_traffic(client, world):
    actors = traffic_actors(world)
    for actor in actors:
        if actor.type_id.startswith('sensor.') or actor.type_id.startswith('controller.'):
            try:
                actor.stop()
            except Exception:
                pass
    destroy_actors(client, actors)
    return len(actors)


def main():
    parser = argparse.ArgumentParser(description='Spawn NPC traffic, an ego vehicle, and pedestrians')
    parser.add_argument('--host', default='127.0.0.1', help='CARLA host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port (default: 2000)')
    parser.add_argument('--tm-port', type=int, default=8000, help='Traffic Manager port (default: 8000)')
    parser.add_argument('-n', '--vehicles', type=int, default=50, help='Number of NPC vehicles (default: 50)')
    parser.add_argument('-w', '--walkers', type=int, default=20, help='Number of pedestrians (default: 20)')
    parser.add_argument('--no-autopilot', action='store_true', help='Spawn vehicles parked, without Traffic Manager')
    parser.add_argument('--ego-autopilot', action='store_true', help='Also put the ego vehicle on autopilot')
    parser.add_argument('--follow', action='store_true', help='Keep the spectator camera behind the ego vehicle')
    parser.add_argument('--output', default='_out', help='Folder for RGB camera PNGs (default: _out)')
    parser.add_argument('--no-record', action='store_true', help='Do not attach the ego RGB camera')
    args = parser.parse_args()

    client, world = connect(args.host, args.port)
    map_name = world.get_map().name.replace('/Game/Carla/Maps/', '').split('/')[-1]
    print('Connected. Current map: %s' % map_name)

    leftover = clear_traffic(client, world)
    if leftover:
        world.wait_for_tick()
        print('Removed %d leftover vehicles, walkers, and sensors' % leftover)

    npc_vehicles = []
    ego_vehicle = None
    camera = None
    walkers = []
    controllers = []

    try:
        npc_vehicles, spawn_points = spawn_npc_vehicles(world, args.vehicles)
        print('Spawned %d / %d NPC vehicles' % (len(npc_vehicles), args.vehicles))

        ego_vehicle = spawn_ego_vehicle(world, spawn_points, len(npc_vehicles))
        print('Spawned ego vehicle: %s (id=%d)' % (ego_vehicle.type_id, ego_vehicle.id))
        follow_ego(world, ego_vehicle)

        if not args.no_record:
            output_dir = args.output
            if not os.path.isabs(output_dir):
                repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                output_dir = os.path.join(repo_root, output_dir)
            camera = attach_rgb_camera(world, ego_vehicle, output_dir)
            print('RGB camera recording to %s' % output_dir)

        walkers, controllers = spawn_walkers(world, args.walkers)
        print('Spawned %d / %d pedestrians' % (len(walkers), args.walkers))

        if not args.no_autopilot:
            traffic_manager = client.get_trafficmanager(args.tm_port)
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            for vehicle in npc_vehicles:
                vehicle.set_autopilot(True, traffic_manager.get_port())
            if args.ego_autopilot:
                ego_vehicle.set_autopilot(True, traffic_manager.get_port())
            print('NPC autopilot enabled via Traffic Manager')
        else:
            print('Autopilot disabled; vehicles are parked')

        if args.ego_autopilot:
            print('Ego is on autopilot. Press Ctrl+C to exit.')
            while True:
                world.wait_for_tick()
                if args.follow:
                    follow_ego(world, ego_vehicle)
        else:
            drive_ego(world, ego_vehicle, follow=True)

    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        print('Destroying spawned actors...')
        try:
            if pygame.get_init():
                pygame.quit()
            if camera is not None:
                camera.stop()
            time.sleep(0.5)
            for controller in controllers:
                if controller is not None:
                    controller.stop()
            removed = clear_traffic(client, world)
            time.sleep(0.25)
            print('Removed %d actors' % removed)
        except Exception as exc:
            print('Cleanup failed: %s' % exc)
        print('Done.')


if __name__ == '__main__':
    main()
