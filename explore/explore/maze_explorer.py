#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
import random

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.robot_radius = 0.5
        self.target_reached_tol = 1.0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10
        )

        # Publishers
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        # Nav2 Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self.costmap = None
        self.frontiers = []
        self.current_goal = None

        self.create_timer(1.0, self.explore)

        self.get_logger().info("Frontier Explorer started.")

    # ---------------- Costmap ----------------
    def costmap_callback(self, msg):
        self.costmap = msg
        self.detect_frontiers()

    # ---------------- Robot Pose ----------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ---------------- Frontier Detection ----------------
    def detect_frontiers(self):
        if self.costmap is None:
            return
        data = np.array(self.costmap.data).reshape(
            self.costmap.info.height, self.costmap.info.width
        )
        frontiers = []
        h, w = data.shape
        for y in range(1, h-1):
            for x in range(1, w-1):
                if data[y, x] == -1 and np.any(data[y-1:y+2, x-1:x+2] == 0):
                    frontiers.append((x, y))
        self.frontiers = frontiers
        self.publish_frontiers()

    # ---------------- Convert ----------------
    def cell_to_world(self, x, y):
        res = self.costmap.info.resolution
        origin = self.costmap.info.origin.position
        return origin.x + (x + 0.5) * res, origin.y + (y + 0.5) * res

    # ---------------- Visualization ----------------
    def publish_frontiers(self):
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.frontiers):
            wx, wy = self.cell_to_world(x, y)
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
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

    # ---------------- Send Goal ----------------
    def send_goal(self, wx, wy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal)
        self.current_goal = (wx, wy)

    # ---------------- Exploration ----------------
    def explore(self):
        if self.costmap is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry = pose

        # Remove reached frontiers
        if self.current_goal:
            gx, gy = self.current_goal
            if math.hypot(gx - rx, gy - ry) < self.target_reached_tol:
                self.current_goal = None
                self.frontiers = [f for f in self.frontiers if self.cell_to_world(*f) != (gx, gy)]

        if self.current_goal or not self.frontiers:
            return

        # ---------------- Probabilistic frontier selection ----------------
        # Weight: far frontiers slightly more likely, but keep some near
        distances = [math.hypot(rx - self.cell_to_world(x, y)[0],
                                ry - self.cell_to_world(x, y)[1])
                     for x, y in self.frontiers]
        # Avoid division by zero
        weights = [d + 0.1 for d in distances]
        total = sum(weights)
        probs = [w / total for w in weights]

        selected = random.choices(self.frontiers, weights=probs, k=1)[0]
        wx, wy = self.cell_to_world(*selected)
        self.send_goal(wx, wy)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
