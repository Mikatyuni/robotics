import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class PositionTrackerNode(Node):
    def __init__(self):
        super().__init__('position_tracker_node')
        pkg_share = get_package_share_directory('office_delivery_robot')
        rooms_path = os.path.join(pkg_share, 'config', 'office_rooms.yaml')
        with open(rooms_path, 'r') as f:
            self.office_layout = yaml.safe_load(f)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pose = None
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

        self.pose_pub = self.create_publisher(
            PoseStamped, '/office_bot/tracked_pose', 10)
        self.status_pub = self.create_publisher(
            String, '/office_bot/location_status', 10)

        self.timer = self.create_timer(0.5, self.report_position)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom_pose = (p.x, p.y, yaw)

    def classify_zone(self, world_x: float, world_y: float):
        for room in self.office_layout.get('rooms', []):
            x_min, y_min, x_max, y_max = room['bounds']
            if x_min <= world_x <= x_max and y_min <= world_y <= y_max:
                return room['id'], room['name'], room['access']

        corr = self.office_layout.get('corridor', {}).get('bounds', [-12.9, -1.5, 12.9, 1.5])
        if corr[0] <= world_x <= corr[2] and corr[1] <= world_y <= corr[3]:
            return 'corridor', 'Main Office Corridor', 'allowed'

        for room in self.office_layout.get('rooms', []):
            dx, dy = room['door_center']
            if math.hypot(world_x - dx, world_y - dy) <= 0.8:
                return f"door_{room['id']}", f"Doorway to {room['name']}", room['access']

        return 'transition', 'Office Transition Space', 'allowed'

    def report_position(self):
        if self.odom_pose is None:
            return

        # With OdometryPublisher, /odom is directly in gazebo_world coordinates
        world_x, world_y, yaw = self.odom_pose
        mx, my, myaw = world_x + 1.0, world_y, yaw
        frame_used = 'odom'

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            mx = tf.transform.translation.x
            my = tf.transform.translation.y
            myaw = yaw_from_quat(tf.transform.rotation)
            frame_used = 'map'
        except TransformException:
            pass

        yaw_deg = math.degrees(myaw)
        zone_id, zone_name, access = self.classify_zone(world_x, world_y)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_used
        pose_msg.pose.position.x = float(mx)
        pose_msg.pose.position.y = float(my)
        pose_msg.pose.orientation.z = math.sin(myaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(myaw / 2.0)
        self.pose_pub.publish(pose_msg)

        status_data = {
            'frame': frame_used,
            'map_x': round(mx, 3),
            'map_y': round(my, 3),
            'world_x': round(world_x, 3),
            'world_y': round(world_y, 3),
            'yaw_deg': round(yaw_deg, 2),
            'zone_id': zone_id,
            'zone_name': zone_name,
            'access': access,
        }
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

        if access == 'forbidden':
            self.get_logger().warning(
                f'[RESTRICTED ZONE] World=({world_x:.2f}, {world_y:.2f}) | '
                f'Map=({mx:.2f}, {my:.2f}) | Yaw={yaw_deg:.1f} deg | Zone={zone_name}',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f'World=({world_x:.2f}, {world_y:.2f}) | Map=({mx:.2f}, {my:.2f}) | '
                f'Yaw={yaw_deg:.1f} deg | Zone={zone_name} ({access})',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PositionTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
