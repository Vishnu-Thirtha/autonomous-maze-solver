#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from collections import deque
import math
import time


class TurningAssist(Node):

    def __init__(self):
        super().__init__('turning_assist')

        # ---------------- Parameters ----------------
        self.stuck_time = 2.0                # seconds to consider robot stuck
        self.movement_threshold = 0.5       # meters
        self.assist_start_fraction = 0.8     # start with 80% of plan
        self.assist_fraction_step = 0.05     # reduce 5% each attempt
        self.min_assist_fraction = 0.05      # never go below 5%

        # ---------------- Subscribers ----------------
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.odom_callback, 10)

        # ---------------- Action Client ----------------
        self.client = ActionClient(self, FollowPath, 'follow_path')

        # ---------------- State ----------------
        self.original_plan = None
        self.last_robot_pose = None
        self.assist_active = False
        self.current_fraction = self.assist_start_fraction

        # For stuck detection
        self.last_checked_pose = None
        self.last_checked_time = None

        # ---------------- Timer ----------------
        self.create_timer(0.15, self.monitor)

        self.get_logger().info("TurningAssist node started")

    # ---------------- Callbacks ----------------
    def path_callback(self, msg: Path):
        if not msg.poses:
            return
        self.original_plan = msg
        self.assist_active = False
        self.current_fraction = self.assist_start_fraction
        self.get_logger().info(f"Received global plan with {len(msg.poses)} poses")

    def odom_callback(self, msg: PoseStamped):
        self.last_robot_pose = (msg.pose.position.x, msg.pose.position.y)
        if self.last_checked_pose is None:
            self.last_checked_pose = self.last_robot_pose
            self.last_checked_time = time.time()

    # ---------------- Monitor ----------------
    def monitor(self):
        if self.original_plan is None:
            # Only warn once
            if not hasattr(self, "_warned_no_plan"):
                self.get_logger().warn("No global plan received yet")
                self._warned_no_plan = True
            return

        if self.last_robot_pose is None:
            # Only warn once
            if not hasattr(self, "_warned_no_odom"):
                self.get_logger().warn("No odometry received yet")
                self._warned_no_odom = True
            return

        # Reset warnings once both plan and odom exist
        if hasattr(self, "_warned_no_plan"):
            del self._warned_no_plan
        if hasattr(self, "_warned_no_odom"):
            del self._warned_no_odom

        # ---------------- Check if robot is stuck ----------------
        if self.robot_stuck():
            if not self.assist_active:
                self.get_logger().warn("Robot stuck — starting assist")
                self.assist_active = True
                self.current_fraction = self.assist_start_fraction
            self.attempt_assist()
        else:
            if self.assist_active:
                self.get_logger().info("Robot moved — resuming original plan")
                self.send_subgoal(self.original_plan)
                self.assist_active = False
                self.current_fraction = self.assist_start_fraction


    # ---------------- Robot Stuck Check ----------------
    def robot_stuck(self):
        """
        Returns True if the robot has not made significant movement over
        `stuck_time` AND the global plan is not progressing.
        """

        if self.last_robot_pose is None or self.original_plan is None:
            return False

        now = time.time()

        # --- Track robot movement ---
        if not hasattr(self, "_pose_history"):
            self._pose_history = deque(maxlen=2)  # store last 5 poses
            self._time_history = deque(maxlen=2)
        
        self._pose_history.append(self.last_robot_pose)
        self._time_history.append(now)

        if len(self._pose_history) < 2:
            return False

        # Compute distance moved over last few poses
        x0, y0 = self._pose_history[0]
        x1, y1 = self._pose_history[-1]
        dist_moved = math.hypot(x1 - x0, y1 - y0)
        time_elapsed = self._time_history[-1] - self._time_history[0]

        if time_elapsed < self.stuck_time:
            return False

        # --- Track repeated plan lengths ---
        if not hasattr(self, "_plan_history"):
            self._plan_history = deque(maxlen=5)  # store last 5 plan lengths

        if self.original_plan:
            self._plan_history.append(len(self.original_plan.poses))

        repeated_plan = len(set(self._plan_history)) == 1 if len(self._plan_history) == self._plan_history.maxlen else False

        # Robot considered stuck if distance is small and plan is repeating
        if dist_moved < 0.2 and repeated_plan:  # 0.2 m threshold
            self.get_logger().warn(f"Robot stuck detected: moved {dist_moved:.3f}m over {time_elapsed:.1f}s, repeated plan")
            return True

        return False

    # ---------------- Assist Logic ----------------
    def attempt_assist(self):
        total_poses = len(self.original_plan.poses)
        fraction = self.current_fraction

        while fraction >= self.min_assist_fraction:
            index = max(1, int(total_poses * fraction)) - 1
            truncated_plan = Path()
            truncated_plan.header = self.original_plan.header
            truncated_plan.poses = self.original_plan.poses[:index + 1]

            self.get_logger().info(
                f"Assist: sending {fraction*100:.0f}% of plan "
                f"({index+1}/{total_poses} poses)"
            )
            self.send_subgoal(truncated_plan)

            # brief wait to see if robot moves
            time.sleep(0.5)

            if not self.robot_stuck():
                self.get_logger().info("Assist successful — robot started moving")
                return

            fraction -= self.assist_fraction_step

        self.get_logger().error("Assist failed — robot still stuck")

    # ---------------- Send Subgoal ----------------
    def send_subgoal(self, path: Path):
        if not path.poses:
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("FollowPath action server not available")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        self.client.send_goal_async(goal_msg)


# ---------------- Main ----------------
def main():
    rclpy.init()
    node = TurningAssist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
