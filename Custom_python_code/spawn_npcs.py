"""Populate the current CARLA map with NPC vehicles, an ego vehicle, and pedestrians.

CarlaUE4.exe must already be running (use load_map.py first if you need a town).

The ego vehicle is the actor an autonomous agent would control. NPC vehicles use
Traffic Manager autopilot. Pedestrians walk to random navigation points.

Press Ctrl+C to destroy spawned actors and exit.

Examples:
    python spawn_npcs.py
    python spawn_npcs.py --vehicles 50 --walkers 20
    python spawn_npcs.py --no-autopilot
    python spawn_npcs.py --follow
"""

import argparse
import random
import time

import carla


def connect(host, port, timeout=30.0):
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
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    leftover = spawn_points[used_count:] or spawn_points
    random.shuffle(leftover)

    for transform in leftover:
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            blueprint.set_attribute(
                'color', random.choice(blueprint.get_attribute('color').recommended_values)
            )
        blueprint.set_attribute('role_name', 'hero')
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


def destroy_actors(actors):
    for actor in actors:
        if actor is not None and actor.is_alive:
            actor.destroy()


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
    args = parser.parse_args()

    client, world = connect(args.host, args.port)
    map_name = world.get_map().name.replace('/Game/Carla/Maps/', '').split('/')[-1]
    print('Connected. Current map: %s' % map_name)

    npc_vehicles = []
    ego_vehicle = None
    walkers = []
    controllers = []

    try:
        npc_vehicles, spawn_points = spawn_npc_vehicles(world, args.vehicles)
        print('Spawned %d / %d NPC vehicles' % (len(npc_vehicles), args.vehicles))

        ego_vehicle = spawn_ego_vehicle(world, spawn_points, len(npc_vehicles))
        print('Spawned ego vehicle: %s (id=%d)' % (ego_vehicle.type_id, ego_vehicle.id))
        follow_ego(world, ego_vehicle)

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

        print('Simulation populated. Press Ctrl+C to destroy actors and exit.')
        while True:
            world.wait_for_tick()
            if args.follow:
                follow_ego(world, ego_vehicle)

    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        print('Destroying spawned actors...')
        for controller in controllers:
            if controller is not None and controller.is_alive:
                controller.stop()
        destroy_actors(controllers + walkers + npc_vehicles + [ego_vehicle])
        time.sleep(0.25)
        print('Done.')


if __name__ == '__main__':
    main()
