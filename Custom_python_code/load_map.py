"""Load a CARLA map from the client.

CarlaUE4.exe must already be running.

Examples:
    python load_map.py --list
    python load_map.py --map Town05
    python load_map.py --reload
"""

import argparse
import sys

import carladddda


def short_map_name(name):
    return name.replace('/Game/Carla/Maps/', '').split('/')[-1]


def current_map_name(world):
    return short_map_name(world.get_map().name)


def available_maps(client):
    return sorted(short_map_name(m) for m in client.get_available_maps())


def main():
    parser = argparse.ArgumentParser(description='Load or inspect the current CARLA map')
    parser.add_argument('--host', default='127.0.0.1', help='CARLA host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port (default: 2000)')
    parser.add_argument('-m', '--map', help='Map to load, e.g. Town05')
    parser.add_argument('-l', '--list', action='store_true', help='List available maps')
    parser.add_argument('-r', '--reload', action='store_true', help='Reload the current map')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    maps = available_maps(client)
    world = client.get_world()
    print('Current map: %s' % current_map_name(world))

    if args.list:
        print('\nAvailable maps:')
        for name in maps:
            print('  %s' % name)
        return

    if args.reload:
        print('Reloading map...')
        world = client.reload_world()
        print('Loaded %s' % current_map_name(world))
        return

    if args.map:
        if args.map not in maps:
            print('Unknown map %r. Use --list to see available maps.' % args.map, file=sys.stderr)
            sys.exit(1)
        print('Loading %s...' % args.map)
        world = client.load_world(args.map)
        print('Loaded %s' % current_map_name(world))
        return

    print('Use --list to see maps, or --map Town05 to load one.')


if __name__ == '__main__':
    main()
