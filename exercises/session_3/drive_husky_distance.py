#!/usr/bin/env python3
"""Drive the Husky straight backward until odometry measures a set distance."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ReverseHusky(Node):
    def __init__(self, distance, speed, timeout):
        super().__init__("drive_husky_distance")
        self.distance = distance
        self.speed = speed
        self.timeout = timeout
        self.started_at = time.monotonic()
        self.start_pose = None
        self.current_xy = None
        self.last_odom_at = None
        self.finished = False
        self.success = False

        self.publisher = self.create_publisher(
            Twist, "/husky_runner/cmd_vel", 10
        )
        self.subscription = self.create_subscription(
            Odometry, "/husky_runner/odom", self._on_odom, 10
        )
        self.timer = self.create_timer(0.1, self._step)

    def _on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.current_xy = (position.x, position.y)
        self.last_odom_at = time.monotonic()
        if self.start_pose is None:
            self.start_pose = (position.x, position.y, yaw)
            self.get_logger().info(
                f"Starting odometry x={position.x:.3f}, y={position.y:.3f}, "
                f"yaw={yaw:.3f} rad; reversing {self.distance:.2f} m."
            )

    def _publish_velocity(self, linear_x):
        command = Twist()
        command.linear.x = linear_x
        self.publisher.publish(command)

    def _step(self):
        now = time.monotonic()
        if now - self.started_at > self.timeout:
            self.get_logger().error(
                f"Timed out before reaching {self.distance:.2f} m."
            )
            self.finished = True
            return

        if self.start_pose is None:
            return
        if self.last_odom_at is None or now - self.last_odom_at > 1.0:
            self.get_logger().error("Odometry stopped updating; stopping the Husky.")
            self.finished = True
            return

        start_x, start_y, start_yaw = self.start_pose
        delta_x = self.current_xy[0] - start_x
        delta_y = self.current_xy[1] - start_y
        backward_distance = -(
            delta_x * math.cos(start_yaw) + delta_y * math.sin(start_yaw)
        )
        if backward_distance >= self.distance:
            self._publish_velocity(0.0)
            self.get_logger().info(
                f"Reached {backward_distance:.3f} m backward; stopping."
            )
            self.success = True
            self.finished = True
            return

        self._publish_velocity(-self.speed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, default=0.50, help="metres")
    parser.add_argument("--speed", type=float, default=0.08, help="m/s")
    parser.add_argument("--timeout", type=float, default=20.0, help="wall seconds")
    args = parser.parse_args()
    if args.distance <= 0 or args.speed <= 0 or args.timeout <= 0:
        parser.error("distance, speed, and timeout must all be positive")

    rclpy.init()
    node = ReverseHusky(args.distance, args.speed, args.timeout)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted; sending a stop command.")
    finally:
        for _ in range(3):
            node._publish_velocity(0.0)
            time.sleep(0.05)
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
