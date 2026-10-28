"""Send the robot to several rooms in an optimised order using Nav2.

AI-assisted. After every stop the remaining stops are re-ordered from the
robot's current pose, so a blocked or aborted stop does not stall the route.
Aborted stops are retried after the other stops, up to max_attempts times.
"""

import math

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from office_delivery_robot.delivery_route import (
    Stop, load_layout, optimize_route, stops_from_layout, world_to_map)
from office_delivery_robot.position_tracker_node import yaw_from_quat


class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')
        self.declare_parameter('rooms', ['hr', 'accounting', 'reception', 'it'])
        self.declare_parameter('return_to_start', False)
        self.declare_parameter('dwell_sec', 2.0)
        self.declare_parameter('goal_timeout_sec', 300.0)
        self.declare_parameter('max_attempts', 2)
        self.declare_parameter('map_offset_x', 1.0)
        self.declare_parameter('map_offset_y', 0.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def robot_pose(self, timeout_sec=10.0):
        """Return (x, y, yaw) of base_footprint in the map frame, or None."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            except TransformException:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, yaw_from_quat(q)
        return None

    def go_to(self, stop):
        """Navigate to a stop (map-frame coordinates). True if Nav2 reports success."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = stop.x
        goal.pose.pose.position.y = stop.y
        goal.pose.pose.orientation.z = math.sin(stop.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(stop.yaw / 2.0)

        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'Nav2 rejected the goal for {stop.name}')
            return False

        timeout = float(self.get_parameter('goal_timeout_sec').value)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if not result.done():
            self.get_logger().error(f'Timed out going to {stop.name}; cancelling')
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=5.0)
            return False
        return result.result().status == GoalStatus.STATUS_SUCCEEDED

    def run(self):
        share = get_package_share_directory('office_delivery_robot')
        layout = load_layout(os.path.join(share, 'config', 'office_rooms.yaml'))
        offset = (float(self.get_parameter('map_offset_x').value),
                  float(self.get_parameter('map_offset_y').value))
        return_home = bool(self.get_parameter('return_to_start').value)
        max_attempts = int(self.get_parameter('max_attempts').value)
        dwell = float(self.get_parameter('dwell_sec').value)

        world_stops, rejected = stops_from_layout(
            layout, list(self.get_parameter('rooms').value))
        for room_id, reason in rejected:
            self.get_logger().warning(f'Skipping {room_id}: {reason}')
        stops = []
        for s in world_stops:
            mx, my = world_to_map(s.x, s.y, *offset)
            stops.append(Stop(s.room_id, s.name, mx, my, s.yaw))
        if not stops:
            self.get_logger().error('No deliverable rooms requested')
            return False

        if not self.nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False
        pose = self.robot_pose()
        if pose is None:
            self.get_logger().error('No map -> base_footprint transform')
            return False
        home = pose

        attempts = {s.room_id: 0 for s in stops}
        remaining, deferred, delivered, failed = list(stops), [], [], []
        while remaining or deferred:
            if not remaining:
                remaining, deferred = deferred, []
            order, length = optimize_route(
                pose[:2], remaining, return_to_start=return_home)
            self.get_logger().info(
                'Route: ' + ' -> '.join(s.room_id for s in order) + f' ({length:.1f} m)')
            stop = order[0]
            remaining.remove(stop)
            attempts[stop.room_id] += 1
            if self.go_to(stop):
                self.get_logger().info(f'Delivered to {stop.name}')
                delivered.append(stop.room_id)
                self.sleep(dwell)
            elif attempts[stop.room_id] < max_attempts:
                self.get_logger().warning(f'{stop.name} blocked or failed; retrying later')
                deferred.append(stop)
            else:
                self.get_logger().error(f'Giving up on {stop.name}')
                failed.append(stop.room_id)
            pose = self.robot_pose(timeout_sec=2.0) or (stop.x, stop.y, stop.yaw)

        if return_home:
            self.go_to(Stop('home', 'start position', home[0], home[1], home[2]))
        self.get_logger().info(f'Delivered: {delivered} | Failed: {failed}')
        return not failed

    def sleep(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
