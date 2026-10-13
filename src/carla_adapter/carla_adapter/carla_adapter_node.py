import math

import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class CarlaAdapter(Node):

    def __init__(self):
        super().__init__('carla_adapter')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('host', '172.30.240.1')
        self.declare_parameter('port', 2000)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        # ============================================================
        # CONNECT TO CARLA
        # ============================================================

        self.get_logger().info(
            f'Connecting to CARLA at {host}:{port}...'
        )

        self.client = carla.Client(
            host,
            port
        )

        self.client.set_timeout(10.0)

        try:

            self.world = self.client.get_world()

            self.get_logger().info(
                'Connected to CARLA 0.9.16'
            )

            self.get_logger().info(
                f'Map: {self.world.get_map().name}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Could not connect to CARLA: {e}'
            )

            raise

        # ============================================================
        # EGO VEHICLE
        # ============================================================

        self.vehicle = None
        self.owns_vehicle = False

        self.vehicle = self.find_or_spawn_ego()

        # ============================================================
        # ROS 2 POSE PUBLISHER
        # ============================================================

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/carla/ego/pose',
            10
        )

        # ============================================================
        # PUBLISH AT 20 Hz
        # ============================================================

        self.timer = self.create_timer(
            0.05,
            self.publish_pose
        )

        # ============================================================
        # STATUS
        # ============================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'CARLA -> ROS 2 adapter is running!'
        )

        self.get_logger().info(
            'Publishing: /carla/ego/pose'
        )

        self.get_logger().info(
            'Ego vehicle is ready.'
        )

        self.get_logger().info(
            '========================================'
        )

    # ================================================================
    # FIND EXISTING EGO OR SPAWN A NEW ONE
    # ================================================================

    def find_or_spawn_ego(self):

        # ------------------------------------------------------------
        # Check whether an ego vehicle already exists.
        # ------------------------------------------------------------

        vehicles = self.world.get_actors().filter(
            'vehicle.*'
        )

        for vehicle in vehicles:

            role_name = vehicle.attributes.get(
                'role_name',
                ''
            )

            if role_name == 'ego':

                self.get_logger().info(
                    f'Found existing ego vehicle '
                    f'(ID: {vehicle.id})'
                )

                return vehicle

        # ------------------------------------------------------------
        # No ego vehicle exists.
        # ------------------------------------------------------------

        self.get_logger().info(
            'No ego vehicle found.'
        )

        self.get_logger().info(
            'Searching for a valid road spawn...'
        )

        blueprint_library = (
            self.world.get_blueprint_library()
        )

        vehicle_bp = blueprint_library.find(
            'vehicle.tesla.model3'
        )

        vehicle_bp.set_attribute(
            'role_name',
            'ego'
        )

        # ------------------------------------------------------------
        # Get CARLA's predefined spawn points.
        # ------------------------------------------------------------

        spawn_points = (
            self.world.get_map().get_spawn_points()
        )

        if not spawn_points:

            raise RuntimeError(
                'CARLA map contains no spawn points.'
            )

        self.get_logger().info(
            f'CARLA provided '
            f'{len(spawn_points)} spawn points.'
        )

        # ------------------------------------------------------------
        # Find spawn points that correspond to actual
        # driving lanes.
        # ------------------------------------------------------------

        road_candidates = []

        for spawn_point in spawn_points:

            waypoint = (
                self.world.get_map().get_waypoint(
                    spawn_point.location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )
            )

            if waypoint is None:
                continue

            # Don't spawn directly inside an intersection.
            if waypoint.is_junction:
                continue

            # Make absolutely sure it is a driving lane.
            if waypoint.lane_type != carla.LaneType.Driving:
                continue

            road_candidates.append(
                waypoint
            )

        self.get_logger().info(
            f'Found {len(road_candidates)} '
            f'valid driving-road candidates.'
        )

        if not road_candidates:

            raise RuntimeError(
                'No valid driving-road spawn points found.'
            )

        # ------------------------------------------------------------
        # Try each candidate until CARLA accepts one.
        # ------------------------------------------------------------

        for index, waypoint in enumerate(
            road_candidates
        ):

            spawn_transform = (
                waypoint.transform
            )

            # Lift the vehicle slightly above the road.
            spawn_transform.location.z += 0.3

            self.get_logger().info(
                f'Trying road spawn '
                f'{index + 1}/'
                f'{len(road_candidates)}: '
                f'x={spawn_transform.location.x:.2f}, '
                f'y={spawn_transform.location.y:.2f}, '
                f'z={spawn_transform.location.z:.2f}, '
                f'yaw={spawn_transform.rotation.yaw:.2f}'
            )

            vehicle = self.world.try_spawn_actor(
                vehicle_bp,
                spawn_transform
            )

            if vehicle is None:
                continue

            # --------------------------------------------------------
            # SUCCESS
            # --------------------------------------------------------

            self.owns_vehicle = True

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                f'Spawned Tesla Model 3 '
                f'on driving road '
                f'(ID: {vehicle.id})'
            )

            location = vehicle.get_location()

            rotation = vehicle.get_transform().rotation

            self.get_logger().info(
                f'Spawn position: '
                f'x={location.x:.2f}, '
                f'y={location.y:.2f}, '
                f'z={location.z:.2f}'
            )

            self.get_logger().info(
                f'Spawn heading: '
                f'yaw={rotation.yaw:.2f} degrees'
            )

            self.get_logger().info(
                '========================================'
            )

            return vehicle

        # ------------------------------------------------------------
        # Nothing worked.
        # ------------------------------------------------------------

        raise RuntimeError(
            'Could not spawn ego vehicle on any '
            'valid driving-road location.'
        )

    # ================================================================
    # CARLA ROTATION -> ROS QUATERNION
    # ================================================================

    def carla_rotation_to_quaternion(
        self,
        rotation
    ):

        roll = math.radians(
            rotation.roll
        )

        pitch = math.radians(
            rotation.pitch
        )

        yaw = math.radians(
            rotation.yaw
        )

        cy = math.cos(
            yaw * 0.5
        )

        sy = math.sin(
            yaw * 0.5
        )

        cp = math.cos(
            pitch * 0.5
        )

        sp = math.sin(
            pitch * 0.5
        )

        cr = math.cos(
            roll * 0.5
        )

        sr = math.sin(
            roll * 0.5
        )

        qw = (
            cr * cp * cy
            + sr * sp * sy
        )

        qx = (
            sr * cp * cy
            - cr * sp * sy
        )

        qy = (
            cr * sp * cy
            + sr * cp * sy
        )

        qz = (
            cr * cp * sy
            - sr * sp * cy
        )

        return qx, qy, qz, qw

    # ================================================================
    # PUBLISH EGO POSE
    # ================================================================

    def publish_pose(self):

        if self.vehicle is None:
            return

        if not self.vehicle.is_alive:

            self.get_logger().warning(
                'Ego vehicle is no longer alive.'
            )

            return

        # ------------------------------------------------------------
        # Get current CARLA transform.
        # ------------------------------------------------------------

        transform = (
            self.vehicle.get_transform()
        )

        # ------------------------------------------------------------
        # Create ROS PoseStamped.
        # ------------------------------------------------------------

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'map'

        # ------------------------------------------------------------
        # Position
        # ------------------------------------------------------------

        msg.pose.position.x = (
            transform.location.x
        )

        msg.pose.position.y = (
            transform.location.y
        )

        msg.pose.position.z = (
            transform.location.z
        )

        # ------------------------------------------------------------
        # Orientation
        # ------------------------------------------------------------

        qx, qy, qz, qw = (
            self.carla_rotation_to_quaternion(
                transform.rotation
            )
        )

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # ------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------

        self.pose_publisher.publish(
            msg
        )

    # ================================================================
    # CLEANUP
    # ================================================================

    def cleanup(self):

        if (
            self.owns_vehicle
            and self.vehicle is not None
            and self.vehicle.is_alive
        ):

            self.get_logger().info(
                f'Destroying ego vehicle '
                f'(ID: {self.vehicle.id})'
            )

            self.vehicle.destroy()

    # ================================================================
    # NODE SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self.cleanup()

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = CarlaAdapter()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f'CARLA adapter error: {e}'
        )

    finally:

        if node is not None:

            node.destroy_node()

        rclpy.shutdown()


# ====================================================================
# ENTRY POINT
# ====================================================================

if __name__ == '__main__':

    main()
