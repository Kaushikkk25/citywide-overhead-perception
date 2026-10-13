import math
import time

import carla
import pygame


# ================================================================
# CONFIGURATION
# ================================================================

CARLA_HOST = '172.30.240.1'
CARLA_PORT = 2000

UPDATE_RATE = 30


# ================================================================
# CAMERA MODES
# ================================================================

FIRST_PERSON = 0
THIRD_PERSON = 1
OVERVIEW = 2


# ================================================================
# CONNECT TO CARLA
# ================================================================

print('Connecting to CARLA...')

client = carla.Client(
    CARLA_HOST,
    CARLA_PORT
)

client.set_timeout(10.0)

world = client.get_world()

print('Connected to CARLA 0.9.16')


# ================================================================
# FIND EGO VEHICLE
# ================================================================

def find_ego():

    vehicles = world.get_actors().filter(
        'vehicle.*'
    )

    for vehicle in vehicles:

        if vehicle.attributes.get(
            'role_name',
            ''
        ) == 'ego':

            return vehicle

    return None


ego = find_ego()

if ego is None:

    print(
        'ERROR: No ego vehicle found.'
    )

    print(
        'Start the CARLA adapter first.'
    )

    raise SystemExit


print(
    f'Ego vehicle found: ID={ego.id}'
)


# ================================================================
# SPECTATOR
# ================================================================

spectator = world.get_spectator()


# ================================================================
# CAMERA MODE
# ================================================================

camera_mode = THIRD_PERSON


# ================================================================
# PYGAME WINDOW
# ================================================================

pygame.init()

screen = pygame.display.set_mode(
    (500, 120)
)

pygame.display.set_caption(
    'CARLA Camera Controller'
)

font = pygame.font.Font(
    None,
    28
)


# ================================================================
# DISPLAY MODE
# ================================================================

def draw_status():

    if camera_mode == FIRST_PERSON:
        mode = 'FIRST PERSON'

    elif camera_mode == THIRD_PERSON:
        mode = 'THIRD PERSON'

    else:
        mode = 'OVERVIEW'

    screen.fill(
        (20, 20, 20)
    )

    text = font.render(
        f'Camera: {mode}',
        True,
        (255, 255, 255)
    )

    controls = font.render(
        'F = First | T = Third | R = Overview | ESC = Exit',
        True,
        (200, 200, 200)
    )

    screen.blit(
        text,
        (20, 15)
    )

    screen.blit(
        controls,
        (20, 60)
    )

    pygame.display.flip()


# ================================================================
# CAMERA UPDATE
# ================================================================

def update_camera():

    global ego

    # ------------------------------------------------------------
    # Re-find ego if necessary.
    # ------------------------------------------------------------

    if ego is None or not ego.is_alive:

        ego = find_ego()

        if ego is None:
            return

    transform = ego.get_transform()

    location = transform.location

    rotation = transform.rotation

    forward = transform.get_forward_vector()

    # ============================================================
    # FIRST PERSON
    # ============================================================

    if camera_mode == FIRST_PERSON:

        # Driver-eye position.
        camera_location = carla.Location(
            x=location.x
                + forward.x * 0.8,

            y=location.y
                + forward.y * 0.8,

            z=location.z
                + 1.6
        )

        camera_rotation = carla.Rotation(
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            roll=0.0
        )

        spectator.set_transform(
            carla.Transform(
                camera_location,
                camera_rotation
            )
        )

    # ============================================================
    # THIRD PERSON
    # ============================================================

    elif camera_mode == THIRD_PERSON:

        # Camera behind the vehicle.
        distance = 8.0

        height = 3.2

        camera_location = carla.Location(
            x=location.x
                - forward.x * distance,

            y=location.y
                - forward.y * distance,

            z=location.z
                + height
        )

        # Look toward the vehicle.
        dx = (
            location.x
            - camera_location.x
        )

        dy = (
            location.y
            - camera_location.y
        )

        dz = (
            location.z
            + 1.0
            - camera_location.z
        )

        horizontal_distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        yaw = math.degrees(
            math.atan2(
                dy,
                dx
            )
        )

        pitch = math.degrees(
            math.atan2(
                dz,
                horizontal_distance
            )
        )

        camera_rotation = carla.Rotation(
            pitch=pitch,
            yaw=yaw,
            roll=0.0
        )

        spectator.set_transform(
            carla.Transform(
                camera_location,
                camera_rotation
            )
        )

    # ============================================================
    # OVERVIEW
    # ============================================================

    elif camera_mode == OVERVIEW:

        camera_location = carla.Location(
            x=location.x,
            y=location.y,
            z=25.0
        )

        camera_rotation = carla.Rotation(
            pitch=-90.0,
            yaw=rotation.yaw,
            roll=0.0
        )

        spectator.set_transform(
            carla.Transform(
                camera_location,
                camera_rotation
            )
        )


# ================================================================
# MAIN LOOP
# ================================================================

clock = pygame.time.Clock()

running = True

draw_status()

print()
print('========================================')
print('CARLA CAMERA CONTROLLER')
print('========================================')
print('F     -> First person')
print('T     -> Third person')
print('R     -> Overview')
print('ESC   -> Exit camera controller')
print('========================================')
print()


while running:

    # ------------------------------------------------------------
    # HANDLE KEYBOARD
    # ------------------------------------------------------------

    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            running = False

        elif event.type == pygame.KEYDOWN:

            if event.key == pygame.K_f:

                camera_mode = FIRST_PERSON

                print(
                    'Camera: FIRST PERSON'
                )

                draw_status()

            elif event.key == pygame.K_t:

                camera_mode = THIRD_PERSON

                print(
                    'Camera: THIRD PERSON'
                )

                draw_status()

            elif event.key == pygame.K_r:

                camera_mode = OVERVIEW

                print(
                    'Camera: OVERVIEW'
                )

                draw_status()

            elif event.key == pygame.K_ESCAPE:

                running = False

    # ------------------------------------------------------------
    # UPDATE CAMERA
    # ------------------------------------------------------------

    update_camera()

    # ------------------------------------------------------------
    # 30 FPS
    # ------------------------------------------------------------

    clock.tick(
        UPDATE_RATE
    )


pygame.quit()

print(
    'Camera controller stopped.'
)
