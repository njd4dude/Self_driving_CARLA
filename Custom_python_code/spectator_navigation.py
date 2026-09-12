"""Move the CARLA spectator (the free camera in the simulator window).

CarlaUE4.exe must already be running. Click the pygame window so keys are captured.

  WASD          move
  Q / E         down / up
  arrows        look
  Shift         faster
  R             jump above the map origin (x=0, y=0)
  P             print current location and rotation
  ESC           quit

Examples:
    python spectator_navigation.py
    python spectator_navigation.py --print
    python spectator_navigation.py --origin
"""

import argparse
import sys

import carla

try:
    import pygame
    from pygame.locals import K_DOWN, K_ESCAPE, K_LEFT, K_LSHIFT, K_RSHIFT
    from pygame.locals import K_RIGHT, K_UP, K_a, K_d, K_e, K_p, K_q, K_r, K_s, K_w
except ImportError:
    raise RuntimeError('pygame is required. Install it with: pip install pygame')


HELP_TEXT = [
    'Spectator navigation',
    '',
    'WASD  move     Q/E  down/up',
    'Arrows look    Shift faster',
    'R  jump to origin   P  print pose',
    'ESC  quit',
]


def print_transform(transform):
    loc = transform.location
    rot = transform.rotation
    print(
        'location: x=%.2f y=%.2f z=%.2f  |  '
        'rotation: pitch=%.1f yaw=%.1f roll=%.1f'
        % (loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll)
    )


def clamp_pitch(pitch):
    return max(-89.0, min(89.0, pitch))


def wrap_yaw(yaw):
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw


def any_fly_key(keys):
    return any(keys[k] for k in (
        K_w, K_a, K_s, K_d, K_q, K_e, K_LEFT, K_RIGHT, K_UP, K_DOWN,
    ))


def origin_transform():
    # (0, 0, 0) is usually underground, so sit above the origin looking down.
    return carla.Transform(
        carla.Location(x=0.0, y=0.0, z=40.0),
        carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0),
    )


def fly_spectator(spectator, dt, keys, speed, look_speed):
    if not any_fly_key(keys):
        return

    transform = spectator.get_transform()
    location = transform.location
    yaw = transform.rotation.yaw
    pitch = transform.rotation.pitch
    roll = transform.rotation.roll

    if keys[K_LEFT]:
        yaw = wrap_yaw(yaw - look_speed * dt)
    if keys[K_RIGHT]:
        yaw = wrap_yaw(yaw + look_speed * dt)
    if keys[K_UP]:
        pitch = clamp_pitch(pitch + look_speed * dt)
    if keys[K_DOWN]:
        pitch = clamp_pitch(pitch - look_speed * dt)

    rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    oriented = carla.Transform(location, rotation)
    forward = oriented.get_forward_vector()
    right = oriented.get_right_vector()
    up = oriented.get_up_vector()

    x, y, z = location.x, location.y, location.z
    move = speed * 3.0 if keys[K_LSHIFT] or keys[K_RSHIFT] else speed
    if keys[K_w]:
        x += forward.x * move * dt
        y += forward.y * move * dt
        z += forward.z * move * dt
    if keys[K_s]:
        x -= forward.x * move * dt
        y -= forward.y * move * dt
        z -= forward.z * move * dt
    if keys[K_d]:
        x += right.x * move * dt
        y += right.y * move * dt
        z += right.z * move * dt
    if keys[K_a]:
        x -= right.x * move * dt
        y -= right.y * move * dt
        z -= right.z * move * dt
    if keys[K_e]:
        x += up.x * move * dt
        y += up.y * move * dt
        z += up.z * move * dt
    if keys[K_q]:
        x -= up.x * move * dt
        y -= up.y * move * dt
        z -= up.z * move * dt

    spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z), rotation))


def draw_help(screen, font):
    screen.fill((20, 20, 24))
    y = 16
    for line in HELP_TEXT:
        color = (230, 230, 230) if line else (20, 20, 24)
        screen.blit(font.render(line, True, color), (16, y))
        y += 28


def run_navigation(world, spectator, speed):
    pygame.init()
    pygame.display.set_caption('CARLA spectator — click this window to fly')
    screen = pygame.display.set_mode((420, 220))
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    look_speed = 90.0

    print('Spectator navigation started. Click the pygame window, then use WASD.')
    print_transform(spectator.get_transform())

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        keys = pygame.key.get_pressed()
        skip_fly = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                elif event.key == K_r:
                    spectator.set_transform(origin_transform())
                    world.wait_for_tick()
                    print('Jumped to map origin (x=0, y=0, z=40, looking down).')
                    print_transform(spectator.get_transform())
                    skip_fly = True
                elif event.key == K_p:
                    print_transform(spectator.get_transform())

        if not skip_fly:
            fly_spectator(spectator, dt, keys, speed, look_speed)
        world.wait_for_tick()

        draw_help(screen, font)
        pygame.display.flip()

    pygame.quit()
    print('Stopped. Final pose:')
    print_transform(spectator.get_transform())


def main():
    parser = argparse.ArgumentParser(description='Control the CARLA spectator camera')
    parser.add_argument('--host', default='127.0.0.1', help='CARLA host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port (default: 2000)')
    parser.add_argument('--print', dest='print_pose', action='store_true',
                        help='Print the current spectator pose and exit')
    parser.add_argument('--origin', action='store_true',
                        help='Teleport spectator to the map origin and exit')
    parser.add_argument('--speed', type=float, default=15.0,
                        help='Move speed in m/s (default: 15, Shift triples it)')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    spectator = world.get_spectator()

    if args.origin:
        spectator.set_transform(origin_transform())
        world.wait_for_tick()
        print('Spectator moved to map origin (x=0, y=0, z=40, looking down).')
        print_transform(spectator.get_transform())
        return

    if args.print_pose:
        print_transform(spectator.get_transform())
        return

    try:
        run_navigation(world, spectator, args.speed)
    except KeyboardInterrupt:
        print('\nStopped.')
        sys.exit(0)


if __name__ == '__main__':
    main()
