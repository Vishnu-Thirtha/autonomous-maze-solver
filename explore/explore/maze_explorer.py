#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
import random
import time


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ---------------- Parameters ----------------
        self.local_explore_prob = 0.33
        self.goal_timeout = 20.0
        self.unknown_cost = 100

        self.min_frontier_dist = 1.0
        self.min_start_dist = 2.0
        self.preferred_dist = 2.5
        self.max_frontier_dist = 8.0
        self.goal_reached_tol = 1.0

        self.distance_weight = 1.2
        self.cost_weight = 0.05
        self.randomness = 0.1

        # ---- NEW: frontier density parameters ----
        self.unknown_window = 3          # radius (cells)
        self.min_unknown_ratio = 0.4     # density threshold

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- ROS ----------------
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.frontier_pub = self.create_publisher(
            MarkerArray, '/frontiers', 10)
        self.current_pub = self.create_publisher(
            Marker, '/current_frontier', 10)

        self.start_sub = self.create_subscription(
            Marker, '/mission/start', self.start_marker_callback, 10)
        self.goal_sub = self.create_subscription(
            Marker, '/mission/goal', self.goal_marker_callback, 10)

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ---------------- State ----------------
        self.costmap = None
        self.frontiers = []
        self.start_pose = None
        self.goal_pose = None
        self.goal_visible = False
        self.current_target = None
        self.goal_start_time = None

        # ---- NEW: goal commitment flag ----
        self.goal_committed = False

        self.create_timer(1.0, self.explore)

        self.get_logger().info("Frontier Explorer (cost-aware, goal-aware) started")

    # ---------------- Callbacks ----------------
    def costmap_callback(self, msg):
        self.costmap = msg
        self.frontiers = self.detect_frontiers()

    def start_marker_callback(self, msg):
        self.start_pose = (msg.pose.position.x, msg.pose.position.y)

    def goal_marker_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)

    # ---------------- Robot Pose ----------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except:
            return None

    # ---------------- Frontier Logic ----------------
    def detect_frontiers(self):
        data = np.array(self.costmap.data).reshape(
            self.costmap.info.height,
            self.costmap.info.width
        )
        frontiers = []
        for y in range(1, data.shape[0]-1):
            for x in range(1, data.shape[1]-1):
                if data[y, x] == -1 and np.any(data[y-1:y+2, x-1:x+2] == 0):
                    frontiers.append((x, y))
        return frontiers

    def cell_to_world(self, x, y):
        res = self.costmap.info.resolution
        org = self.costmap.info.origin.position
        return org.x + (x + 0.5) * res, org.y + (y + 0.5) * res

    def cell_cost(self, x, y):
        idx = y * self.costmap.info.width + x
        cost = self.costmap.data[idx]
        if cost < 0:
            return self.unknown_cost
        return cost

    # ---- NEW: unknown density check ----
    def unknown_density(self, x, y):
        data = np.array(self.costmap.data).reshape(
            self.costmap.info.height,
            self.costmap.info.width
        )
        w = self.unknown_window
        y0 = max(0, y - w)
        y1 = min(data.shape[0], y + w + 1)
        x0 = max(0, x - w)
        x1 = min(data.shape[1], x + w + 1)

        window = data[y0:y1, x0:x1]
        unknowns = np.sum(window == -1)
        total = window.size
        return unknowns / total if total > 0 else 0.0

    # ---------------- Frontier Selection ----------------
    def choose_frontier(self, rx, ry):
        worlds = [self.cell_to_world(x, y) for x, y in self.frontiers]

        if self.start_pose is None:
            self.start_pose = (rx, ry)

        scored = []
        for i, (x, y) in enumerate(self.frontiers):
            wx, wy = worlds[i]
            d_robot = math.hypot(wx - rx, wy - ry)
            d_start = math.hypot(wx - self.start_pose[0], wy - self.start_pose[1])
            cost = self.cell_cost(x, y)

            if d_robot < self.min_frontier_dist:
                continue
            if d_start < self.min_start_dist:
                continue

            # ---- NEW: density filter ----
            if self.unknown_density(x, y) < self.min_unknown_ratio:
                continue

            dist_score = (d_robot/self.preferred_dist if d_robot <= self.preferred_dist
                          else (1.0 + (d_robot - self.preferred_dist)/self.max_frontier_dist))
            score = (
                self.distance_weight * dist_score
                + d_start
                - self.cost_weight * cost
                + random.uniform(-self.randomness, self.randomness)
            )
            scored.append((score, (x, y)))

        if not scored:
            return min(
                self.frontiers,
                key=lambda f: math.hypot(
                    self.cell_to_world(f[0], f[1])[0] - rx,
                    self.cell_to_world(f[0], f[1])[1] - ry
                )
            )
        return max(scored, key=lambda s: s[0])[1]

    # ---------------- Goal / Frontier Action ----------------
    def send_goal(self, wx, wy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal)
        self.current_target = (wx, wy)
        self.goal_start_time = time.time()

    # ---------------- Visualization ----------------
    def publish_frontiers(self):
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.frontiers):
            wx, wy = self.cell_to_world(x, y)
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "all_frontiers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.b = 1.0
            m.color.a = 0.7
            ma.markers.append(m)
        self.frontier_pub.publish(ma)

    def publish_current_target(self):
        if not self.current_target:
            return
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "current_frontier"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.current_target[0]
        m.pose.position.y = self.current_target[1]
        m.pose.position.z = 0.1
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r = 1.0
        m.color.a = 1.0
        self.current_pub.publish(m)

    # ---------------- Exploration Loop ----------------
    def explore(self):
        if self.costmap is None or not self.frontiers:
            return

        pose = self.get_robot_pose()
        if pose is None or self.goal_pose is None:
            return

        rx, ry = pose
        gx, gy = self.goal_pose

        dist_to_goal = math.hypot(gx - rx, gy - ry)

        # ---- NEW: hard termination ----
        if dist_to_goal <= self.goal_reached_tol:
            if not getattr(self, "goal_reached", False):
                self.get_logger().info("🎯 Goal reached — stopping exploration!")
                self.goal_reached = True
            return

        # ---- NEW: commit to goal, suspend exploration ----
        if dist_to_goal <= 2.0 and not self.goal_committed:
            self.get_logger().info("🎯 Goal detected — committing to goal!")
            self.goal_committed = True
            self.send_goal(gx, gy)
            return

        if self.goal_committed:
            return

        # Normal frontier exploration
        if self.current_target:
            dist = math.hypot(self.current_target[0]-rx, self.current_target[1]-ry)
            if dist < 0.8 or time.time() - self.goal_start_time > self.goal_timeout:
                self.current_target = None

        if self.current_target is None:
            fx, fy = self.choose_frontier(rx, ry)
            wx, wy = self.cell_to_world(fx, fy)
            self.send_goal(wx, wy)

        self.publish_frontiers()
        self.publish_current_target()


# ---------------- Main ----------------
def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
