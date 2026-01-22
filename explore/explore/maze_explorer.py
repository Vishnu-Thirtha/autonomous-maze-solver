#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math


class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')

        # Subscribe to the SLAM map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_data = None
        self.exploring = True

        # Maze exit coordinates in map cells
        self.exit_cell = None  # Set later dynamically or hardcoded

        # Timer for exploration loop
        self.timer = self.create_timer(2.0, self.explore)

        self.get_logger().info("Maze Explorer Node Started.")

    # ---------------- Map callback ----------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    # ---------------- Map cell to world coordinates ----------------
    def map_to_world(self, x_cell, y_cell):
        res = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        wx = origin.x + (x_cell + 0.5) * res
        wy = origin.y + (y_cell + 0.5) * res
        return wx, wy

    # ---------------- Frontier detection ----------------
    def detect_frontiers(self):
        if self.map_data is None:
            return []

        data = np.array(self.map_data.data).reshape(
            self.map_data.info.height,
            self.map_data.info.width
        )

        frontiers = []
        for y in range(1, data.shape[0]-1):
            for x in range(1, data.shape[1]-1):
                if data[y, x] == 0:  # free
                    neighbors = data[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:
                        frontiers.append((x, y))
        return frontiers

    # ---------------- Send goal to Nav2 ----------------
    def send_goal(self, wx, wy):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = wx
        goal_msg.pose.pose.position.y = wy
        goal_msg.pose.pose.orientation.w = 1.0  # facing forward

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Goal sent: ({wx:.2f}, {wy:.2f})")

    # ---------------- Exploration loop ----------------
    def explore(self):
        if not self.exploring or self.map_data is None:
            return

        # Detect frontiers
        frontiers = self.detect_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers left. Exploration complete.")
            self.exploring = False
            # Optionally send final path from start to exit here
            return

        # Simple greedy: pick closest frontier to robot
        robot_x, robot_y = 0, 0  # Could use /tf or /odom for real pose
        closest = min(frontiers, key=lambda f: (f[0]-robot_x)**2 + (f[1]-robot_y)**2)
        wx, wy = self.map_to_world(*closest)
        self.send_goal(wx, wy)

        # Optional: check if exit cell is discovered
        if self.exit_cell:
            ex, ey = self.exit_cell
            data = np.array(self.map_data.data).reshape(
                self.map_data.info.height,
                self.map_data.info.width
            )
            if data[ey, ex] == 0:
                self.get_logger().info("Exit discovered! Exploration done.")
                self.exploring = False
                # Here you could send the final start->finish path

# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
