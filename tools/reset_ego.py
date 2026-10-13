import carla


HOST = '172.30.240.1'
PORT = 2000


client = carla.Client(HOST, PORT)
client.set_timeout(10.0)

world = client.get_world()

# ============================================================
# FIND EGO
# ============================================================

vehicles = world.get_actors().filter('vehicle.*')

ego = None

for vehicle in vehicles:

    if vehicle.attributes.get(
        'role_name',
        ''
    ) == 'ego':

        ego = vehicle
        break


if ego is None:

    print('ERROR: No ego vehicle found.')

    exit()


print(
    f'Found ego vehicle: ID={ego.id}'
)


# ============================================================
# FIND VALID ROAD SPAWN
# ============================================================

spawn_points = (
    world.get_map().get_spawn_points()
)

valid_points = []

for spawn_point in spawn_points:

    waypoint = (
        world.get_map().get_waypoint(
            spawn_point.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
    )

    if waypoint is None:
        continue

    if waypoint.is_junction:
        continue

    if waypoint.lane_type != carla.LaneType.Driving:
        continue

    valid_points.append(
        waypoint
    )


if not valid_points:

    print(
        'ERROR: No valid road locations found.'
    )

    exit()


# ============================================================
# USE A ROAD LOCATION
# ============================================================

waypoint = valid_points[0]

transform = waypoint.transform

# Slightly above road surface
transform.location.z += 0.5


print(
    'Resetting ego to:'
)

print(
    f'  X: {transform.location.x:.2f}'
)

print(
    f'  Y: {transform.location.y:.2f}'
)

print(
    f'  Z: {transform.location.z:.2f}'
)

print(
    f'  Yaw: {transform.rotation.yaw:.2f}'
)


# ============================================================
# STOP VEHICLE BEFORE TELEPORTING
# ============================================================

ego.apply_control(
    carla.VehicleControl(
        throttle=0.0,
        brake=1.0,
        steer=0.0
    )
)


# ============================================================
# TELEPORT
# ============================================================

ego.set_transform(
    transform
)


# ============================================================
# RESET PHYSICS
# ============================================================

ego.set_target_velocity(
    carla.Vector3D(
        x=0.0,
        y=0.0,
        z=0.0
    )
)

ego.set_target_angular_velocity(
    carla.Vector3D(
        x=0.0,
        y=0.0,
        z=0.0
    )
)


print(
    'SUCCESS: Ego vehicle reset.'
)

print(
    'Check the CARLA window.'
)
