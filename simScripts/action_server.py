#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


class IsaacTrajectoryServer(Node):

    def __init__(self):
        super().__init__("isaac_trajectory_server")

        # 发布给 Isaac Sim
        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_command",
            10,
        )

        # MoveIt 会连接这个 Action
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        self.get_logger().info("----------------------------------")
        self.get_logger().info("Isaac Trajectory Server Started")
        self.get_logger().info("----------------------------------")

    def execute_callback(self, goal_handle):

        self.get_logger().info("Received trajectory.")

        trajectory = goal_handle.request.trajectory

        previous_time = 0.0

        for point in trajectory.points:

            current_time = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )

            dt = current_time - previous_time

            if dt > 0:
                time.sleep(dt)

            msg = JointState()

            msg.header.stamp = self.get_clock().now().to_msg()

            msg.name = list(trajectory.joint_names)

            msg.position = list(point.positions)

            if len(point.velocities) > 0:
                msg.velocity = list(point.velocities)

            if len(point.effort) > 0:
                msg.effort = list(point.effort)

            self.joint_pub.publish(msg)

            previous_time = current_time

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()

        return result


def main():

    rclpy.init()

    node = IsaacTrajectoryServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()