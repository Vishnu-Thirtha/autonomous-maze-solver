#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
from collections import deque


class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')

        # ---------------- Parameters ----------------
        self.robot_radius = 0.5
        self.cost_threshold = 50
        self.timer_period = 2.0
        self.target_reached_tol = 2.0
        self.oscillation_memory = 5

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Subscribers ----------------
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10
        )

        # ---------------- Publishers ----------------
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)
        self.goal_pub = self.create_publisher(Marker, '/exploration_goal', 10)

        # ---------------- Nav2 Action Client ----------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---------------- State ----------------
        self.costmap = None
        self.exploring = True
        self.current_target = None
        self.recent_targets = deque(maxlen=self.oscillation_memory)
        self.goal_handle = None

        # ✅ NEW: persistent favorable frontier memory
        self.favorable_frontiers = set()

        self.timer = self.create_timer(self.timer_period, self.explore)

        self.get_logger().info("Maze Explorer started.")

    # ---------------- Costmap Callback ----------------
    def costmap_callback(self, msg):
        self.costmap = msg
        self.prune_invalid_frontiers()

    # ---------------- TF Robot Pose ----------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ---------------- Cell → World ----------------
    def cell_to_world(self, x, y):
        res = self.costmap.info.resolution
        origin = self.costmap.info.origin.position
        return (
            origin.x + (x + 0.5) * res,
            origin.y + (y + 0.5) * res
        )

    # ---------------- Detect Favorable Frontiers ----------------
    def detect_favorable_frontiers(self):
        data = np.array(self.costmap.data).reshape(
            self.costmap.info.height,
            self.costmap.info.width
        )

        h, w = data.shape

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if data[y, x] == -1:
                    neighbors = data[y - 1:y + 2, x - 1:x + 2]
                    if np.any(neighbors == -1):
                        self.favorable_frontiers.add((x, y))

    # ---------------- Prune Invalid Frontiers ----------------
    def prune_invalid_frontiers(self):
        if self.costmap is None:
            return

        data = self.costmap.data
        w = self.costmap.info.width

        to_remove = []
        for (x, y) in self.favorable_frontiers:
            idx = y * w + x
            if idx < 0 or idx >= len(data) or data[idx] != -1:
                to_remove.append((x, y))

        for cell in to_remove:
            self.favorable_frontiers.discard(cell)

    # ---------------- Filter Near Robot ----------------
    def filter_near_robot(self, targets):
        pose = self.get_robot_pose()
        if pose is None:
            return targets

        rx, ry = pose
        return [
            (x, y) for (x, y) in targets
            if math.hypot(*(np.array(self.cell_to_world(x, y)) - np.array([rx, ry]))) >= self.robot_radius
        ]

    # ---------------- Visualization ----------------
    def publish_targets(self, targets):
        ma = MarkerArray()

        for i, (x, y) in enumerate(targets):
            wx, wy = self.cell_to_world(x, y)
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.b = 1.0
            m.color.a = 0.8
            ma.markers.append(m)

        self.frontier_pub.publish(ma)

    # ---------------- Send Nav Goal ----------------
    def send_goal(self, wx, wy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

    # ---------------- Exploration Loop ----------------
    def explore(self):
        if not self.exploring or self.costmap is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry = pose

        # ✅ Update frontier memory
        self.detect_favorable_frontiers()

        candidates = list(self.favorable_frontiers)
        candidates = self.filter_near_robot(candidates)

        self.publish_targets(candidates)

        # -------- Target validity check --------
        if self.current_target:
            wx, wy = self.cell_to_world(*self.current_target)
            dist = math.hypot(wx - rx, wy - ry)

            idx = self.cell_to_costmap_index(*self.current_target)
            cost = self.costmap.data[idx] if idx is not None else None

            if dist <= self.target_reached_tol or cost != -1:
                self.recent_targets.append(self.current_target)
                self.favorable_frontiers.discard(self.current_target)
                self.current_target = None

        # -------- Choose new target --------
        if self.current_target is None and candidates:
            scores = []
            for (x, y) in candidates:
                if (x, y) in self.recent_targets:
                    scores.append(-np.inf)
                    continue
                wx, wy = self.cell_to_world(x, y)
                scores.append(math.hypot(wx - rx, wy - ry))

            best = candidates[np.argmax(scores)]
            self.current_target = best
            wx, wy = self.cell_to_world(*best)
            self.send_goal(wx, wy)

    # ---------------- Costmap Index ----------------
    def cell_to_costmap_index(self, x, y):
        w = self.costmap.info.width
        idx = y * w + x
        return idx if 0 <= idx < len(self.costmap.data) else None


def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
