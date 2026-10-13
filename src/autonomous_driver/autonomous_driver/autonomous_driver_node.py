import math

import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class AutonomousDriver(Node):

    def __init__(self):
        super().__init__('autonomous_driver')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('host', '172.30.240.1')
        self.declare_parameter('port', 2000)

        # Keep this LOW for the first test.
        self.declare_parameter('target_speed', 10.0)

        # Distance ahead along the lane.
        self.declare_parameter('lookahead_distance', 8.0)

        # Controller gains.
        self.declare_parameter('heading_gain', 1.2)
        self.declare_parameter('lateral_gain', 0.8)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        self.target_speed = float(
            self.get_parameter(
                'target_speed'
            ).value
        )

        self.lookahead_distance = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        self.heading_gain = float(
            self.get_parameter(
                'heading_gain'
            ).value
        )

        self.lateral_gain = float(
            self.get_parameter(
                'lateral_gain'
            ).value
        )

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

        self.world = self.client.get_world()

        self.get_logger().info(
            'Connected to CARLA 0.9.16'
        )

        self.get_logger().info(
            f'Map: {self.world.get_map().name}'
        )

        # ============================================================
        # FIND EGO
        # ============================================================

        self.vehicle = self.find_ego_vehicle()

        # ============================================================
        # ROS POSE SUBSCRIPTION
        # ============================================================

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/carla/ego/pose',
            self.pose_callback,
            10
        )

        self.pose_received = False

        # ============================================================
        # CONTROL LOOP
        # ============================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'LANE FOLLOWING CONTROLLER READY'
        )

        self.get_logger().info(
            f'Target speed: '
            f'{self.target_speed:.1f} km/h'
        )

        self.get_logger().info(
            f'Lookahead: '
            f'{self.lookahead_distance:.1f} m'
        )

        self.get_logger().info(
            'Waiting for ROS 2 pose...'
        )

        self.get_logger().info(
            '========================================'
        )

    # ================================================================
    # FIND EGO
    # ================================================================

    def find_ego_vehicle(self):

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
                    f'Found ego vehicle '
                    f'(ID: {vehicle.id})'
                )

                return vehicle

        raise RuntimeError(
            'No ego vehicle found. '
            'Start the CARLA adapter first.'
        )

    # ================================================================
    # ROS POSE CALLBACK
    # ================================================================

    def pose_callback(self, msg):

        self.pose_received = True

    # ================================================================
    # GET CURRENT SPEED
    # ================================================================

    def get_speed_kmh(self):

        velocity = self.vehicle.get_velocity()

        speed_ms = math.sqrt(
            velocity.x ** 2
            + velocity.y ** 2
            + velocity.z ** 2
        )

        return speed_ms * 3.6

    # ================================================================
    # GET CURRENT ROAD WAYPOINT
    # ================================================================

    def get_current_waypoint(self):

        location = (
            self.vehicle.get_location()
        )

        waypoint = (
            self.world.get_map().get_waypoint(
                location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
        )

        return waypoint

    # ================================================================
    # FIND WAYPOINT AHEAD
    # ================================================================

    def find_target_waypoint(
        self,
        current_waypoint
    ):

        if current_waypoint is None:
            return None

        waypoint = current_waypoint

        distance = 0.0

        step = 2.0

        while distance < self.lookahead_distance:

            next_waypoints = waypoint.next(
                step
            )

            if not next_waypoints:
                break

            # For this baseline controller,
            # follow the first continuation.
            waypoint = next_waypoints[0]

            distance += step

        return waypoint

    # ================================================================
    # ANGLE NORMALIZATION
    # ================================================================

    def normalize_angle(
        self,
        angle
    ):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ================================================================
    # CALCULATE LANE FOLLOWING STEERING
    # ================================================================

    def calculate_steering(
        self,
        current_waypoint,
        target_waypoint
    ):

        vehicle_transform = (
            self.vehicle.get_transform()
        )

        vehicle_location = (
            vehicle_transform.location
        )

        vehicle_yaw = math.radians(
            vehicle_transform.rotation.yaw
        )

        # ------------------------------------------------------------
        # TARGET POSITION
        # ------------------------------------------------------------

        target_location = (
            target_waypoint.transform.location
        )

        dx = (
            target_location.x
            - vehicle_location.x
        )

        dy = (
            target_location.y
            - vehicle_location.y
        )

        # ------------------------------------------------------------
        # TARGET POINT IN VEHICLE COORDINATES
        # ------------------------------------------------------------

        cos_yaw = math.cos(
            vehicle_yaw
        )

        sin_yaw = math.sin(
            vehicle_yaw
        )

        local_x = (
            cos_yaw * dx
            + sin_yaw * dy
        )

        local_y = (
            -sin_yaw * dx
            + cos_yaw * dy
        )

        # ------------------------------------------------------------
        # ROAD HEADING
        # ------------------------------------------------------------

        road_yaw = math.radians(
            target_waypoint.transform.rotation.yaw
        )

        heading_error = self.normalize_angle(
            road_yaw - vehicle_yaw
        )

        # ------------------------------------------------------------
        # LATERAL ERROR
        #
        # Compare vehicle position against current
        # lane center.
        # ------------------------------------------------------------

        lane_location = (
            current_waypoint.transform.location
        )

        lane_dx = (
            vehicle_location.x
            - lane_location.x
        )

        lane_dy = (
            vehicle_location.y
            - lane_location.y
        )

        # Lane right vector.
        lane_yaw = math.radians(
            current_waypoint.transform.rotation.yaw
        )

        lane_right_x = math.sin(
            lane_yaw
        )

        lane_right_y = -math.cos(
            lane_yaw
        )

        lateral_error = (
            lane_dx * lane_right_x
            + lane_dy * lane_right_y
        )

        # ------------------------------------------------------------
        # COMBINED CONTROLLER
        # ------------------------------------------------------------

        heading_term = (
            self.heading_gain
            * heading_error
        )

        lateral_term = (
            self.lateral_gain
            * lateral_error
        )

        # Lookahead target contributes a small
        # additional correction.
        target_term = 0.15 * local_y

        steering = (
            heading_term
            + lateral_term
            + target_term
        )

        # ------------------------------------------------------------
        # LIMIT STEERING
        # ------------------------------------------------------------

        steering = max(
            -0.45,
            min(0.45, steering)
        )

        return (
            steering,
            heading_error,
            lateral_error,
            local_x,
            local_y
        )

    # ================================================================
    # SPEED CONTROLLER
    # ================================================================

    def calculate_speed_control(
        self
    ):

        speed = self.get_speed_kmh()

        error = (
            self.target_speed
            - speed
        )

        # Conservative throttle.
        throttle = 0.025 * error

        throttle = max(
            0.0,
            min(0.30, throttle)
        )

        brake = 0.0

        # Brake if we're clearly above target speed.
        if error < -2.0:

            throttle = 0.0

            brake = min(
                0.25,
                abs(error) * 0.04
            )

        return (
            throttle,
            brake,
            speed
        )

    # ================================================================
    # STOP VEHICLE
    # ================================================================

    def stop_vehicle(self):

        if (
            self.vehicle is not None
            and self.vehicle.is_alive
        ):

            control = carla.VehicleControl()

            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0

            self.vehicle.apply_control(
                control
            )

    # ================================================================
    # MAIN CONTROL LOOP
    # ================================================================

    def control_loop(self):

        if (
            self.vehicle is None
            or not self.vehicle.is_alive
        ):
            return

        # Don't move until ROS 2 pose is alive.
        if not self.pose_received:
            return

        # ------------------------------------------------------------
        # CURRENT WAYPOINT
        # ------------------------------------------------------------

        current_waypoint = (
            self.get_current_waypoint()
        )

        if current_waypoint is None:

            self.stop_vehicle()

            self.get_logger().warning(
                'Vehicle is not on a driving lane. '
                'STOPPING.'
            )

            return

        # ------------------------------------------------------------
        # TARGET WAYPOINT
        # ------------------------------------------------------------

        target_waypoint = (
            self.find_target_waypoint(
                current_waypoint
            )
        )

        if target_waypoint is None:

            self.stop_vehicle()

            self.get_logger().warning(
                'No target waypoint found. '
                'STOPPING.'
            )

            return

        # ------------------------------------------------------------
        # STEERING
        # ------------------------------------------------------------

        (
            steering,
            heading_error,
            lateral_error,
            local_x,
            local_y
        ) = self.calculate_steering(
            current_waypoint,
            target_waypoint
        )

        # ------------------------------------------------------------
        # SPEED
        # ------------------------------------------------------------

        (
            throttle,
            brake,
            speed
        ) = self.calculate_speed_control()

        # ------------------------------------------------------------
        # CREATE CONTROL
        # ------------------------------------------------------------

        control = carla.VehicleControl()

        control.throttle = throttle
        control.brake = brake
        control.steer = steering

        control.hand_brake = False
        control.reverse = False

        # ------------------------------------------------------------
        # APPLY
        # ------------------------------------------------------------

        self.vehicle.apply_control(
            control
        )

        # ------------------------------------------------------------
        # DEBUG LOGGING
        # ------------------------------------------------------------

        now = self.get_clock().now()

        elapsed = (
            now - self.last_log_time
        ).nanoseconds / 1e9

        if elapsed >= 1.0:

            self.get_logger().info(
                f'Speed={speed:.1f} km/h | '
                f'Steer={steering:.2f} | '
                f'Throttle={throttle:.2f} | '
                f'Brake={brake:.2f} | '
                f'HeadingErr='
                f'{math.degrees(heading_error):.1f}° | '
                f'LaneErr='
                f'{lateral_error:.2f} m'
            )

            self.last_log_time = now

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self.stop_vehicle()

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

        node = AutonomousDriver()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f'Autonomous driver error: {e}'
        )

    finally:

        if node is not None:

            node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
