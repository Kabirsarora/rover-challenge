import math
import random
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import LaserScan


class SimpleNavigator(Node):
    DRIVE_SPEED = 1.15
    MAX_TURN_SPEED = 1.0
    TURN_GAIN = 2.0
    AVOIDANCE_DRIVE_SECONDS = 1.5
    OBSTACLE_IGNORE_SECONDS = 2.0
    WAYPOINT_TOLERANCE = 0.1
    WAYPOINT_HIT_RADIUS = 0.55

    def __init__(self):
        super().__init__('simple_navigator')

        self.cmd_publisher = self.create_publisher(
            Twist, '/model/rover/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(
            Pose, '/rover/pose', self.pose_callback, 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/rover/scan', self.scan_callback, 10)
        self.marker_client = self.create_client(
            SpawnEntity, '/world/rover_map/create')
        self.delete_marker_client = self.create_client(
            DeleteEntity, '/world/rover_map/remove')
        self.timer = self.create_timer(0.1, self.control_loop)

        self.position = None
        self.yaw = None
        self.scan = None
        self.last_status_time = 0.0
        self.waypoint_started_at = None
        self.avoidance_started_at = None
        self.avoidance_direction = None
        self.avoidance_turn_target = None
        self.avoidance_attempts = 0
        self.ignore_obstacles_until = 0.0
        self.markers_spawned = False
        self.pending_marker_deletions = set()
        self.marker_deletions_in_flight = set()

        generator = random.Random()
        self.waypoints = [
            (generator.uniform(-8.0, 8.0), generator.uniform(-8.0, 8.0))
            for _ in range(3)
        ]
        self.remaining_waypoints = list(range(len(self.waypoints)))
        self.current_waypoint = None
        self.phase = 'select_waypoint'
        self.get_logger().info('Generated waypoint coordinates:')
        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(
                f'  waypoint {index + 1}: x={waypoint[0]:.2f}, y={waypoint[1]:.2f}')

    def pose_callback(self, message):
        self.position = (message.position.x, message.position.y)
        self.yaw = self.quaternion_to_yaw(message.orientation)

    def scan_callback(self, message):
        self.scan = message

    def control_loop(self):
        if self.position is None or self.yaw is None:
            self.stop()
            return

        self.delete_pending_waypoint_markers()

        if self.phase == 'complete':
            self.stop()
            return

        if self.phase == 'select_waypoint':
            self.select_closest_waypoint()
            return

        if self.current_waypoint is None:
            return

        self.spawn_waypoint_markers()

        if self.current_waypoint_hit():
            self.complete_current_waypoint('hit')
            return

        if self.waypoint_timed_out():
            self.stop()
            self.get_logger().warn(
                f'Waypoint {self.current_waypoint + 1} timed out and skipped')
            self.remaining_waypoints.remove(self.current_waypoint)
            self.current_waypoint = None
            self.phase = 'select_waypoint'
            return

        if (self.phase in ('move_x', 'move_y') and
                time.monotonic() >= self.ignore_obstacles_until and
                self.front_obstacle_distance() <= 2.0):
            self.stop()
            self.phase = 'avoid_turn_right'
            self.avoidance_turn_target = self.normalize_angle(self.yaw - math.pi / 2.0)
            self.avoidance_started_at = time.monotonic()
            self.avoidance_attempts = 0
            self.get_logger().warn(
                f'Obstacle detected and distance is approximately '
                f'{self.front_obstacle_distance():.2f} m')
            return

        if self.phase == 'avoid_turn_right':
            self.rotate_to(self.avoidance_turn_target)
            if abs(self.angle_error(self.avoidance_turn_target)) < 0.08:
                self.stop()
                self.phase = 'avoid_check_right'
                self.get_logger().info('Checking right side')
            return

        if self.phase == 'avoid_check_right':
            if self.front_obstacle_distance() > 2.0:
                self.get_logger().info('Right side clear')
                self.start_avoidance_drive('right')
            else:
                self.get_logger().warn('Right side blocked')
                self.phase = 'avoid_turn_left'
                self.avoidance_turn_target = self.normalize_angle(self.yaw + math.pi)
            return

        if self.phase == 'avoid_turn_left':
            self.rotate_to(self.avoidance_turn_target)
            if abs(self.angle_error(self.avoidance_turn_target)) < 0.08:
                self.stop()
                self.phase = 'avoid_check_left'
                self.get_logger().info('Checking left side')
            return

        if self.phase == 'avoid_check_left':
            if self.front_obstacle_distance() > 2.0:
                self.get_logger().info('Left side clear')
                self.start_avoidance_drive('left')
            else:
                self.get_logger().warn('Left side blocked; trying another direction')
                self.phase = 'avoid_turn_extra'
                self.avoidance_turn_target = self.normalize_angle(self.yaw + math.pi / 2.0)
            return

        if self.phase == 'avoid_turn_extra':
            self.rotate_to(self.avoidance_turn_target)
            if abs(self.angle_error(self.avoidance_turn_target)) < 0.08:
                self.stop()
                self.phase = 'avoid_check_extra'
                self.get_logger().info('Checking another direction')
            return

        if self.phase == 'avoid_check_extra':
            if self.front_obstacle_distance() > 2.0:
                self.get_logger().info('Another direction is clear')
                self.start_avoidance_drive('extra')
            else:
                self.stop()
                self.get_logger().warn('All checked directions are blocked; waiting')
            return

        if self.phase == 'avoid_drive':
            self.drive_forward()
            if time.monotonic() - self.avoidance_started_at >= self.AVOIDANCE_DRIVE_SECONDS:
                self.stop()
                self.ignore_obstacles_until = time.monotonic() + self.OBSTACLE_IGNORE_SECONDS
                self.phase = 'resume_path'
                self.get_logger().info('Resuming path toward waypoint')
            return

        if self.phase == 'resume_path':
            self.phase = self.next_axis_phase()
            return

        self.move_toward_waypoint()

    def select_closest_waypoint(self):
        if not self.remaining_waypoints:
            self.phase = 'complete'
            self.stop()
            self.get_logger().info('All waypoints completed')
            return

        distances = []
        for index in self.remaining_waypoints:
            distance = self.distance_to(self.waypoints[index])
            distances.append((index, distance))
        self.get_logger().info(
            'Calculated distances to remaining waypoints: ' +
            ', '.join(f'{index + 1}={distance:.2f} m' for index, distance in distances))
        self.current_waypoint = min(distances, key=lambda item: item[1])[0]
        self.waypoint_started_at = time.monotonic()
        self.phase = self.next_axis_phase()
        self.get_logger().info(
            f'Waypoint {self.current_waypoint + 1} selected because it is closest')

    def spawn_waypoint_markers(self):
        if self.markers_spawned or not self.marker_client.service_is_ready():
            return

        selected_index = self.current_waypoint
        for index, waypoint in enumerate(self.waypoints):
            selected = index == selected_index
            color = '1 1 0 1' if selected else '0 1 1 1'
            marker_sdf = f'''<sdf version="1.8">
  <model name="waypoint_marker_{index + 1}">
    <static>true</static>
    <link name="marker_link">
      <visual name="marker_visual">
        <geometry><sphere><radius>{0.2 if selected else 0.14}</radius></sphere></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <emissive>{color}</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
            request = SpawnEntity.Request()
            request.entity_factory.name = f'waypoint_marker_{index + 1}'
            request.entity_factory.sdf = marker_sdf
            request.entity_factory.pose.position.x = waypoint[0]
            request.entity_factory.pose.position.y = waypoint[1]
            request.entity_factory.pose.position.z = 0.2
            request.entity_factory.relative_to = 'world'
            future = self.marker_client.call_async(request)
            future.add_done_callback(
                lambda result, marker_index=index: self.marker_spawned_callback(
                    result, marker_index))
        self.markers_spawned = True

    def marker_spawned_callback(self, future, index):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Waypoint {index + 1} marker spawned in Gazebo')
            else:
                self.get_logger().warn(
                    f'Waypoint {index + 1} marker was not spawned')
        except Exception as error:
            self.get_logger().error(
                f'Could not spawn waypoint {index + 1} marker: {error}')

    def move_toward_waypoint(self):
        target = self.waypoints[self.current_waypoint]
        distance = self.distance_to(target)
        self.log_status(f'Current rover position: x={self.position[0]:.2f}, y={self.position[1]:.2f}')
        self.log_status(f'Distance remaining: {distance:.2f} m')

        if distance <= self.WAYPOINT_TOLERANCE:
            self.complete_current_waypoint('reached')
            return

        if self.phase == 'move_x':
            target_heading = 0.0 if target[0] > self.position[0] else math.pi
            self.get_logger().debug('Moving along X axis')
        else:
            target_heading = math.pi / 2.0 if target[1] > self.position[1] else -math.pi / 2.0
            self.get_logger().debug('Moving along Y axis')

        if abs(self.angle_error(target_heading)) > 0.08:
            self.rotate_to(target_heading)
        else:
            self.drive_forward()

        if self.phase == 'move_x' and abs(target[0] - self.position[0]) <= self.WAYPOINT_TOLERANCE:
            self.stop()
            self.phase = 'move_y'
        elif self.phase == 'move_y' and abs(target[1] - self.position[1]) <= self.WAYPOINT_TOLERANCE:
            self.complete_current_waypoint('reached')

    def current_waypoint_hit(self):
        return self.distance_to(self.waypoints[self.current_waypoint]) <= self.WAYPOINT_HIT_RADIUS

    def complete_current_waypoint(self, reason):
        waypoint_index = self.current_waypoint
        self.stop()
        self.get_logger().info(
            f'Waypoint {waypoint_index + 1} {reason}; removing marker')
        self.queue_waypoint_marker_deletion(waypoint_index)
        self.remaining_waypoints.remove(waypoint_index)
        self.current_waypoint = None
        self.phase = 'select_waypoint'

    def queue_waypoint_marker_deletion(self, index):
        self.pending_marker_deletions.add(index)
        self.delete_pending_waypoint_markers()

    def delete_pending_waypoint_markers(self):
        for index in list(self.pending_marker_deletions):
            self.delete_waypoint_marker(index)

    def delete_waypoint_marker(self, index):
        if index in self.marker_deletions_in_flight:
            return

        if not self.delete_marker_client.service_is_ready():
            self.get_logger().warn(
                f'Could not remove waypoint {index + 1} marker; remove service is not ready')
            return

        request = DeleteEntity.Request()
        request.entity.name = f'waypoint_marker_{index + 1}'
        request.entity.type = 2
        self.marker_deletions_in_flight.add(index)
        future = self.delete_marker_client.call_async(request)
        future.add_done_callback(
            lambda result, marker_index=index: self.marker_deleted_callback(
                result, marker_index))

    def marker_deleted_callback(self, future, index):
        try:
            response = future.result()
            if response.success:
                self.pending_marker_deletions.discard(index)
                self.get_logger().info(
                    f'Waypoint {index + 1} marker removed from Gazebo')
            else:
                self.get_logger().warn(
                    f'Waypoint {index + 1} marker was not removed')
        except Exception as error:
            self.get_logger().error(
                f'Could not remove waypoint {index + 1} marker: {error}')
        finally:
            self.marker_deletions_in_flight.discard(index)

    def next_axis_phase(self):
        target = self.waypoints[self.current_waypoint]
        if abs(target[0] - self.position[0]) > self.WAYPOINT_TOLERANCE:
            return 'move_x'
        return 'move_y'

    def start_avoidance_drive(self, direction):
        self.avoidance_direction = direction
        self.avoidance_started_at = time.monotonic()
        self.phase = 'avoid_drive'
        self.get_logger().info(f'Moving {direction} to begin going around the obstacle')

    def waypoint_timed_out(self):
        return time.monotonic() - self.waypoint_started_at > 20.0

    def front_obstacle_distance(self):
        if self.scan is None or not self.scan.ranges:
            return float('inf')
        closest = float('inf')
        for index, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if abs(self.normalize_angle(angle)) <= math.radians(15.0):
                if math.isfinite(distance) and distance >= self.scan.range_min:
                    closest = min(closest, distance)
        return closest

    def rotate_to(self, target_heading):
        error = self.angle_error(target_heading)
        command = Twist()
        command.angular.z = max(
            -self.MAX_TURN_SPEED,
            min(self.MAX_TURN_SPEED, self.TURN_GAIN * error))
        self.cmd_publisher.publish(command)
        self.get_logger().debug('Rotating to face a direction')

    def drive_forward(self):
        command = Twist()
        command.linear.x = self.DRIVE_SPEED
        self.cmd_publisher.publish(command)

    def stop(self):
        self.cmd_publisher.publish(Twist())

    def distance_to(self, target):
        return math.hypot(target[0] - self.position[0], target[1] - self.position[1])

    def log_status(self, message):
        now = time.monotonic()
        if now - self.last_status_time >= 1.0:
            self.get_logger().info(message)
            self.last_status_time = now

    def angle_error(self, target_heading):
        return self.normalize_angle(target_heading - self.yaw)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    try:
        rclpy.spin(navigator)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            navigator.stop()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
