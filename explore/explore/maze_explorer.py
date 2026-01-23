#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
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
        self.robot_radius = 0.3        # meters
        self.cost_threshold = 50       # ignore inflated/lethal
        self.timer_period = 2.0        # exploration loop
        self.target_reached_tol = 0.25 # meters to consider target reached
        self.oscillation_memory = 5    # number of recent targets to avoid oscillation

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

        # Timer for exploration loop
        self.timer = self.create_timer(self.timer_period, self.explore)

        self.get_logger().info("Maze Explorer started.")

    # ---------------- Callbacks ----------------
    def costmap_callback(self, msg):
        self.costmap = msg

    # ---------------- TF Robot Pose ----------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ---------------- Map Cell → World ----------------
    def cell_to_world(self, x, y):
        res = self.costmap.info.resolution
        origin = self.costmap.info.origin.position
        wx = origin.x + (x + 0.5) * res
        wy = origin.y + (y + 0.5) * res
        return wx, wy

    # ---------------- Detect Boundary Unknown Cells ----------------
    def detect_boundary_targets(self):
        if self.costmap is None:
            return []

        data = np.array(self.costmap.data).reshape(
            self.costmap.info.height, self.costmap.info.width
        )
        h, w = data.shape
        candidates = []

        for y in range(1, h-1):
            for x in range(1, w-1):
                if data[y, x] == -1:
                    neighbors = data[y-1:y+2, x-1:x+2]
                    if 0 in neighbors:  # free neighbor → boundary
                        candidates.append((x, y))
        return candidates

    # ---------------- Filter targets too close to robot ----------------
    def filter_near_robot(self, targets):
        pose = self.get_robot_pose()
        if pose is None:
            return targets
        rx, ry = pose
        filtered = []
        for (x, y) in targets:
            wx, wy = self.cell_to_world(x, y)
            if math.hypot(wx - rx, wy - ry) >= self.robot_radius:
                filtered.append((x, y))
        return filtered

    # ---------------- Convert cell → costmap index ----------------
    def cell_to_costmap_index(self, x_cell, y_cell):
        if self.costmap is None:
            return None
        res = self.costmap.info.resolution
        origin = self.costmap.info.origin.position
        wx, wy = self.cell_to_world(x_cell, y_cell)
        cx = int((wx - origin.x) / res)
        cy = int((wy - origin.y) / res)
        if cx < 0 or cy < 0 or cx >= self.costmap.info.width or cy >= self.costmap.info.height:
            return None
        return cy * self.costmap.info.width + cx

    # ---------------- Publish Targets ----------------
    def publish_targets(self, targets):
        ma = MarkerArray()
        for i, (x, y) in enumerate(targets):
            wx, wy = self.cell_to_world(x, y)
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "targets"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.b = 1.0
            m.color.a = 0.8
            ma.markers.append(m)
        self.frontier_pub.publish(ma)

    # ---------------- Publish Goal ----------------
    def publish_goal_marker(self, wx, wy):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r = 1.0
        m.color.a = 1.0
        self.goal_pub.publish(m)

    # ---------------- Send Goal ----------------
    def send_goal(self, wx, wy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal)
        self.get_logger().info(f"Exploring → ({wx:.2f}, {wy:.2f})")

    # ---------------- Exploration Loop ----------------
    def explore(self):
        if not self.exploring or self.costmap is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry = pose

        # ---------------- Detect candidate targets ----------------
        candidates = self.detect_boundary_targets()
        candidates = self.filter_near_robot(candidates)

        # ---------------- Publish candidate markers ----------------
        self.publish_targets(candidates)

        # ---------------- Check if current target reached ----------------
        if self.current_target:
            wx, wy = self.cell_to_world(*self.current_target)
            dist = math.hypot(wx - rx, wy - ry)
            idx = self.cell_to_costmap_index(*self.current_target)
            target_cost = None
            if idx is not None:
                target_cost = self.costmap.data[idx]

            if dist <= self.target_reached_tol or \
               target_cost is None or target_cost >= self.cost_threshold:
                self.get_logger().info(f"Target at {self.current_target} reached or blocked.")
                self.recent_targets.append(self.current_target)
                self.current_target = None  # allow choosing a new target

        # ---------------- Pick new target if none ----------------
        if self.current_target is None and candidates:
            # ---------------- Simple scoring: farthest from robot + unknown neighbors ----------------
            data = np.array(self.costmap.data).reshape(
                self.costmap.info.height, self.costmap.info.width
            )
            scores = []
            for (x, y) in candidates:
                if (x, y) in self.recent_targets:
                    scores.append(-np.inf)  # avoid oscillation
                    continue
                wx, wy = self.cell_to_world(x, y)
                dist = math.hypot(wx - rx, wy - ry)
                neighbors = data[max(0, y-1):y+2, max(0, x-1):x+2]
                unknown_count = np.sum(neighbors == -1)
                score = dist + unknown_count
                scores.append(score)

            best_idx = np.argmax(scores)
            self.current_target = candidates[best_idx]

            wx, wy = self.cell_to_world(*self.current_target)
            self.send_goal(wx, wy)
            self.publish_goal_marker(wx, wy)


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
