#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
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
        self.local_explore_prob = 0.33     # 2/3 local, 1/3 global
        self.goal_timeout = 20.0          # seconds
        self.unknown_cost = 100           # penalty for unknown cells
        
        self.min_frontier_dist = 1.0      # meters (hard reject zone)
        self.preferred_dist = 2.5         # meters (sweet spot)
        self.max_frontier_dist = 8.0      # meters (beyond = diminishing returns)

        self.distance_weight = 1.2
        self.cost_weight = 0.05
        self.randomness = 0.1

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

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ---------------- State ----------------
        self.costmap = None
        self.frontiers = []
        self.start_pose = None
        self.current_target = None
        self.goal_start_time = None

        self.create_timer(1.0, self.explore)

        self.get_logger().info("Frontier Explorer (cost-aware) started")

    # -------------------------------------------------------
    def costmap_callback(self, msg):
        self.costmap = msg
        self.frontiers = self.detect_frontiers()

    # -------------------------------------------------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except:
            return None

    # -------------------------------------------------------
    def detect_frontiers(self):
        data = np.array(self.costmap.data).reshape(
            self.costmap.info.height,
            self.costmap.info.width
        )

        frontiers = []
        for y in range(1, data.shape[0] - 1):
            for x in range(1, data.shape[1] - 1):
                if data[y, x] == -1 and np.any(data[y-1:y+2, x-1:x+2] == 0):
                    frontiers.append((x, y))
        return frontiers

    # -------------------------------------------------------
    def cell_to_world(self, x, y):
        res = self.costmap.info.resolution
        org = self.costmap.info.origin.position
        return org.x + (x + 0.5) * res, org.y + (y + 0.5) * res

    # -------------------------------------------------------
    def cell_cost(self, x, y):
        w = self.costmap.info.width
        idx = y * w + x
        cost = self.costmap.data[idx]
        if cost < 0:
            return self.unknown_cost
        return cost

    # -------------------------------------------------------
    def choose_frontier(self, rx, ry):
        worlds = [self.cell_to_world(x, y) for x, y in self.frontiers]

        if self.start_pose is None:
            self.start_pose = (rx, ry)

        scored = []

        for i, (x, y) in enumerate(self.frontiers):
            wx, wy = worlds[i]

            d_robot = math.hypot(wx - rx, wy - ry)
            d_start = math.hypot(wx - self.start_pose[0],
                                wy - self.start_pose[1])

            cost = self.cell_cost(x, y)

            # ---------- 1. Hard reject very close points ----------
            if d_robot < self.min_frontier_dist:
                continue

            # ---------- 2. Distance reward curve ----------
            if d_robot <= self.preferred_dist:
                # Encourage outward motion
                dist_score = d_robot / self.preferred_dist
            elif d_robot <= self.max_frontier_dist:
                # Strong preference for far points
                dist_score = 1.0 + (d_robot - self.preferred_dist) / self.max_frontier_dist
            else:
                # Too far → diminishing returns
                dist_score = 1.5

            # ---------- 3. Start expansion bias ----------
            expand_score = d_start

            # ---------- 4. Cost penalty ----------
            cost_penalty = self.cost_weight * cost

            # ---------- 5. Final score ----------
            score = (
                self.distance_weight * dist_score +
                expand_score -
                cost_penalty +
                random.uniform(-self.randomness, self.randomness)
            )

            scored.append((score, (x, y)))

        # ---------- Fallback: if all rejected, allow closest ----------
        if not scored:
            return min(
                self.frontiers,
                key=lambda f: math.hypot(
                    self.cell_to_world(f[0], f[1])[0] - rx,
                    self.cell_to_world(f[0], f[1])[1] - ry
                )
            )

        # Pick best
        return max(scored, key=lambda s: s[0])[1]


    # -------------------------------------------------------
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

    # -------------------------------------------------------
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

    # -------------------------------------------------------
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

        # 🔴 reduced radius
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r = 1.0
        m.color.a = 1.0

        self.current_pub.publish(m)

    # -------------------------------------------------------
    def explore(self):
        if self.costmap is None or not self.frontiers:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry = pose

        # ---------- Replan if stuck ----------
        if self.current_target:
            dist = math.hypot(
                self.current_target[0] - rx,
                self.current_target[1] - ry
            )
            if dist < 0.8 or time.time() - self.goal_start_time > self.goal_timeout:
                self.current_target = None

        # ---------- Visualize all frontiers ----------
        self.publish_frontiers()

        # ---------- Pick new frontier ----------
        if self.current_target is None:
            fx, fy = self.choose_frontier(rx, ry)
            wx, wy = self.cell_to_world(fx, fy)
            self.send_goal(wx, wy)

        self.publish_current_target()


# -------------------------------------------------------
def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
