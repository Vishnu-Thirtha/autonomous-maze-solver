#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import math
import time
import numpy as np

CELL_SIZE = 1.0  # meters per cell

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ---------------- Parameters ----------------
        self.goal_reached_tol = 0.4   # meters
        self.rows = self.declare_parameter("rows", 15).value
        self.cols = self.declare_parameter("cols", 15).value

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Publishers ----------------
        self.start_pub = self.create_publisher(Marker, '/mission/start', 1)
        self.goal_pub = self.create_publisher(Marker, '/mission/goal', 1)
        self.status_pub = self.create_publisher(Bool, '/mission/status', 1)

        # ---------------- Subscribers ----------------
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # ---------------- State ----------------
        self.map = None
        self.start_pose = None   # in meters
        self.goal_pose = None    # in meters
        self.start_time = time.time()
        self.goal_announced = False

        # ---------------- Timer ----------------
        self.create_timer(1.0, self.update)

        self.get_logger().info(f"Mission Manager started with maze dim: {self.rows} x {self.cols}")

    # ---------------- Map callback ----------------
    def map_callback(self, msg):
        if self.map is None:
            self.map = msg
            self.initialize_start_goal()

    # ---------------- Initialize start/goal ----------------
    def initialize_start_goal(self):
        # Align with robot spawn (leftmost column, top row)
        x_spawn = 0 * CELL_SIZE + CELL_SIZE / 2
        y_spawn = (self.rows - 1) * CELL_SIZE + CELL_SIZE / 2
        self.start_pose = (x_spawn, y_spawn)

        # Goal: rightmost column, bottom row
        x_goal = (self.cols - 1) * CELL_SIZE 
        y_goal = 0 * CELL_SIZE 
        self.goal_pose = (x_goal, y_goal)

        self.publish_goal_marker()
        self.get_logger().info(
            f"Start pose: {self.start_pose} m, Goal pose: {self.goal_pose} m"
        )

    # ---------------- Robot pose ----------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except:
            return None

    # ---------------- Publish markers ----------------
    def publish_start_marker(self):
        self.start_pub.publish(self.make_marker(*self.start_pose, 0.0, 1.0, 0.0, "start"))

    def publish_goal_marker(self):
        self.goal_pub.publish(self.make_marker(*self.goal_pose, 1.0, 0.0, 0.0, "goal"))

    def make_marker(self, x, y, r, g, b, ns):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        return m

    # ---------------- Map metrics ----------------
    def compute_explored_percentage(self):
        if self.map is None:
            return 0.0
        data = np.array(self.map.data)
        explored = np.sum(data >= 0)
        total = data.size
        return 100.0 * explored / total

    # ---------------- Update loop ----------------
    def update(self):
        if self.map is None:
            return

        # Always publish start marker
        self.publish_start_marker()

        # Always publish goal marker
        self.publish_goal_marker()

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry = pose
        gx, gy = self.goal_pose
        dist = math.hypot(rx - gx, ry - gy)
        explored_pct = self.compute_explored_percentage()
        elapsed = time.time() - self.start_time

        # Log metrics only when goal reached
        if dist <= self.goal_reached_tol and not self.goal_announced:
            self.goal_announced = True
            self.status_pub.publish(Bool(data=True))
            self.get_logger().info("🎯 GOAL REACHED — Mission Complete")
            self.get_logger().info(
                f"Time taken: {elapsed:.2f}s | "
                f"Map explored: {explored_pct:.1f}% | "
                f"Goal distance: {dist:.2f} m"
            )


# ---------------- Main ----------------
def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
