"""Smooth chase-cam window for the ego vehicle spawned by manual_control.py.

Look at THIS pygame window. The Unreal spectator cannot be attached to a car,
so a second Python script teleporting it will always look like the car is
sliding forward and back (blurry plate). That is not the physics ragdolling.

Drive in manual_control.py. Run this in another terminal. Click the pygame
window only if you need to quit (ESC).

Also: restart manual_control.py — it now copies its own attached camera into
the Unreal spectator each frame. Stop this script if you only care about Unreal,
otherwise the two clients fight over the spectator.

Examples:
    python follow_ego.py
    python follow_ego.py --distance 10 --height 4 --pitch -15
"""

import argparse
import sys
import time
import weakref

import carla
import numpy as np

try:
    import pygame
    from pygame.locals import K_ESCAPE
except ImportError:
    raise RuntimeError('pygame is required. Install it with: pip install pygame')


VIEW_WIDTH = 1280
VIEW_HEIGHT = 720


class CameraView(object):
    __slots__ = ('surface', '__weakref__')

    def __init__(self):
        self.surface = None


def find_ego(world, actor_id=None, role_name='hero'):
    vehicles = list(world.get_actors().filter('vehicle.*'))
    if actor_id is not None:
        actor = world.get_actor(actor_id)
        if actor is None or not actor.type_id.startswith('vehicle.'):
            raise RuntimeError('No vehicle with id %d' % actor_id)
        return actor

    heroes = [v for v in vehicles if v.attributes.get('role_name') == role_name]
    if heroes:
        return heroes[0]
    if len(vehicles) == 1:
        return vehicles[0]
    return None


def wait_for_ego(world, actor_id, role_name, timeout):
    deadline = time.time() + timeout
    while True:
        ego = find_ego(world, actor_id=actor_id, role_name=role_name)
        if ego is not None:
            return ego
        if timeout >= 0 and time.time() >= deadline:
            raise RuntimeError(
                'No ego vehicle found (role_name=%r). Start manual_control.py first.'
                % role_name
            )
        print('Waiting for ego vehicle...')
        world.wait_for_tick()
        time.sleep(0.25)


def attach_chase_camera(world, vehicle, distance, height, pitch):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
    camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
    if camera_bp.has_attribute('motion_blur_intensity'):
        camera_bp.set_attribute('motion_blur_intensity', '0.0')
    if camera_bp.has_attribute('enable_postprocess_effects'):
        camera_bp.set_attribute('enable_postprocess_effects', 'False')

    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(
            carla.Location(x=-abs(distance), z=height),
            carla.Rotation(pitch=pitch),
        ),
        attach_to=vehicle,
        attachment_type=carla.AttachmentType.SpringArmGhost,
    )

    view = CameraView()
    weak_view = weakref.ref(view)

    def on_image(image):
        holder = weak_view()
        if holder is None:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        holder.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    camera.listen(on_image)
    return camera, view


def destroy_camera(camera):
    if camera is None:
        return
    try:
        camera.stop()
    except Exception:
        pass
    try:
        camera.destroy()
    except Exception:
        pass


def follow_loop(world, ego, distance, height, pitch):
    pygame.init()
    pygame.display.set_caption('CARLA chase cam — attached (smooth). Look here, not Unreal.')
    screen = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()

    camera, view = attach_chase_camera(world, ego, distance, height, pitch)
    print(
        'Attached chase camera on %s (id=%d). Watch the pygame window.'
        % (ego.type_id, ego.id)
    )

    running = True
    try:
        while running:
            clock.tick_busy_loop(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == K_ESCAPE:
                    running = False

            try:
                alive = ego.is_alive
            except RuntimeError:
                alive = False
            if not alive:
                print('Ego vehicle is gone. Waiting for a new one...')
                return

            if view.surface is not None:
                screen.blit(view.surface, (0, 0))
            else:
                screen.fill((20, 20, 24))
            screen.blit(font.render('Attached chase cam  |  ESC quit', True, (230, 230, 230)), (16, 16))
            pygame.display.flip()
    finally:
        destroy_camera(camera)
        if pygame.get_init():
            pygame.quit()


def main():
    parser = argparse.ArgumentParser(description='Attached chase-cam window for the ego vehicle')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--id', type=int, default=None)
    parser.add_argument('--role', default='hero')
    parser.add_argument('--distance', type=float, default=8.0)
    parser.add_argument('--height', type=float, default=3.5)
    parser.add_argument('--pitch', type=float, default=-12.0)
    parser.add_argument('--wait', type=float, default=60.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        while True:
            ego = wait_for_ego(world, args.id, args.role, args.wait)
            follow_loop(world, ego, args.distance, args.height, args.pitch)
            if args.id is not None:
                break
    except KeyboardInterrupt:
        print('\nStopped following.')
        sys.exit(0)


if __name__ == '__main__':
    main()
