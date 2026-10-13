import carla
import time

client = carla.Client('172.30.240.1', 2000)
client.set_timeout(10.0)

world = client.get_world()

vehicles = world.get_actors().filter('vehicle.*')

ego = None

for vehicle in vehicles:
    if vehicle.attributes.get('role_name', '') == 'ego':
        ego = vehicle
        break

if ego is None:
    print('No ego vehicle found.')
    exit()

transform = ego.get_transform()

print(
    f'Ego found: ID={ego.id} '
    f'Location=({transform.location.x:.2f}, '
    f'{transform.location.y:.2f}, '
    f'{transform.location.z:.2f})'
)

# Put spectator behind and above the vehicle
spectator_location = transform.location - (
    transform.get_forward_vector() * 8.0
)

spectator_location.z += 4.0

spectator_rotation = transform.rotation

spectator = world.get_spectator()

spectator.set_transform(
    carla.Transform(
        spectator_location,
        spectator_rotation
    )
)

print('Camera moved to ego vehicle.')
