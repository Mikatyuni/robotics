"""Time the running Nav2 global planner on fixed room-to-room queries.

AI-assisted. Run once with nav2_astar.yaml and once with nav2_dijkstra.yaml
(same start/goal pairs both times) and compare the printed means or the CSVs.
Only the planner server is queried; the robot does not move.
"""

import csv
import itertools
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from office_delivery_robot.delivery_route import (
    load_layout, stops_from_layout, world_to_map)


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def path_length(path):
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class PlannerBenchmark(Node):
    def __init__(self):
        super().__init__('planner_benchmark')
        self.declare_parameter('rooms', ['hr', 'accounting', 'manager', 'reception', 'it'])
        self.declare_parameter('repeats', 5)
        self.declare_parameter('label', 'run')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('output', '')
        self.declare_parameter('map_offset_x', 1.0)
        self.declare_parameter('map_offset_y', 0.0)
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def query(self, start, goal):
        msg = ComputePathToPose.Goal()
        msg.start = make_pose(start.x, start.y, start.yaw)
        msg.goal = make_pose(goal.x, goal.y, goal.yaw)
        msg.use_start = True
        msg.planner_id = str(self.get_parameter('planner_id').value)
        send = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=30.0)
        if not result.done() or not result.result().result.path.poses:
            return None
        res = result.result().result
        ms = res.planning_time.sec * 1e3 + res.planning_time.nanosec / 1e6
        return ms, path_length(res.path)

    def run(self):
        share = get_package_share_directory('office_delivery_robot')
        layout = load_layout(os.path.join(share, 'config', 'office_rooms.yaml'))
        offset = (float(self.get_parameter('map_offset_x').value),
                  float(self.get_parameter('map_offset_y').value))
        world_stops, _ = stops_from_layout(layout, list(self.get_parameter('rooms').value))
        stops = [s._replace(x=world_to_map(s.x, s.y, *offset)[0],
                            y=world_to_map(s.x, s.y, *offset)[1]) for s in world_stops]
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('compute_path_to_pose action server not available')
            return

        label = str(self.get_parameter('label').value)
        repeats = int(self.get_parameter('repeats').value)
        rows = []
        for a, b in itertools.combinations(stops, 2):
            samples = [self.query(a, b) for _ in range(repeats)]
            samples = [s for s in samples if s is not None]
            if not samples:
                self.get_logger().warning(f'No path {a.room_id} -> {b.room_id}')
                continue
            ms = sum(s[0] for s in samples) / len(samples)
            rows.append((label, a.room_id, b.room_id, round(ms, 2), round(samples[0][1], 2)))
            print(f'{label}: {a.room_id:>10} -> {b.room_id:<10} '
                  f'plan {ms:7.2f} ms  path {samples[0][1]:6.2f} m')
        if rows:
            print(f'{label}: mean plan {sum(r[3] for r in rows) / len(rows):.2f} ms, '
                  f'mean path {sum(r[4] for r in rows) / len(rows):.2f} m '
                  f'over {len(rows)} queries')
        output = str(self.get_parameter('output').value)
        if output and rows:
            os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
            with open(output, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['label', 'from', 'to', 'mean_plan_ms', 'path_m'])
                writer.writerows(rows)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerBenchmark()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
