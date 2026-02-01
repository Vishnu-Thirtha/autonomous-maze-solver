#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math


class TurningAssist(Node):

    def __init__(self):
        super().__init__('turning_assist')

        self.goal_reached_tol = 0.4  # meters

        # ---------------- Subscribers ----------------
        self.plan_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)

        # ---------------- Action Client ----------------
        self.client = ActionClient(self, FollowPath, 'follow_path')
        self.goal_handle = None

        # ---------------- State ----------------
        self.original_plan = None
        self.robot_pose = None
        self.assist_active = False
        self.assist_handled = False  # <-- new flag to only handle first stuck event
        self.current_subgoal = None

        self.get_logger().info("TurningAssist started")

    # ---------------- Callbacks ----------------
    def plan_callback(self, msg: Path):
        if not msg.poses:
            return
        self.original_plan = msg
        self.assist_active = False
        self.assist_handled = False
        self.current_subgoal = None

        self.get_logger().info(f"Received global plan ({len(msg.poses)} poses)")

        # Send full plan first
        self.send_plan(msg)

    def pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg.pose.position

    # ---------------- Action Handling ----------------
    def send_plan(self, path: Path):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("FollowPath server unavailable")
            return

        goal = FollowPath.Goal()
        goal.path = path

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected")
            return

        # <<< Listen to result for Nav2 “no progress” feedback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

        self.goal_handle = goal_handle

    def goal_result_callback(self, future):
        status = future.result().status

        # ---- Handle success ----
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.handle_success()

        # ---- Handle first failure only ----
        elif status == GoalStatus.STATUS_ABORTED:
            if not self.assist_handled:
                self.handle_failure()
                self.assist_handled = True  # <-- mark that we reacted

    # ---------------- Assist logic ----------------
    def handle_failure(self):
        self.get_logger().warn("Nav2 reports: FAILED TO MAKE PROGRESS")
        self.assist_active = True

        # Compute mid-turn index (90 deg)
        mid_index = self.find_mid_turn_index()
        if mid_index is None:
            self.get_logger().warn("Path too short or turn too small — retry full plan")
            self.send_plan(self.original_plan)
            return

        # Create truncated path up to mid-turn
        truncated = Path()
        truncated.header = self.original_plan.header
        truncated.poses = self.original_plan.poses[:mid_index + 1]

        self.current_subgoal = truncated.poses[-1].pose.position

        self.get_logger().info(f"Assist: truncated path up to 90deg turn at pose {mid_index + 1}")
        self.send_plan(truncated)

    def handle_success(self):
        # Reset assist_handled if robot made progress
        self.assist_handled = False

        if self.assist_active and self.current_subgoal is not None:
            if self.subgoal_reached():
                self.get_logger().info("Assist complete — resuming original goal")
                self.assist_active = False
                self.replan_to_original_goal()
        else:
            self.get_logger().info("Plan execution succeeded")

    # ---------------- Geometry Helpers ----------------
    def yaw_between(self, p1, p2):
        dx = p2.position.x - p1.position.x
        dy = p2.position.y - p1.position.y
        return math.atan2(dy, dx)

    def find_mid_turn_index(self):
        poses = self.original_plan.poses
        if len(poses) < 3:
            return None

        total_turn = 0
        for i in range(1, len(poses)-1):
            yaw_prev = self.yaw_between(poses[i-1].pose, poses[i].pose)
            yaw_next = self.yaw_between(poses[i].pose, poses[i+1].pose)
            d_yaw = yaw_next - yaw_prev
            # wrap to [-pi, pi]
            d_yaw = (d_yaw + math.pi) % (2*math.pi) - math.pi
            total_turn += abs(d_yaw)
            if total_turn >= math.pi/2:  # 90 degrees
                return i+1
        return None

    def subgoal_reached(self):
        if self.robot_pose is None or self.current_subgoal is None:
            return False
        dx = self.robot_pose.x - self.current_subgoal.x
        dy = self.robot_pose.y - self.current_subgoal.y
        return math.hypot(dx, dy) < self.goal_reached_tol

    def replan_to_original_goal(self):
        final_pose = self.original_plan.poses[-1]
        new_plan = Path()
        new_plan.header = self.original_plan.header
        new_plan.poses = [final_pose]
        self.send_plan(new_plan)


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
