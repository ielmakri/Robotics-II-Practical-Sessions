#!/usr/bin/env python3
"""Publish synthetic joint feedback while demonstrating ROS 2 parameters."""

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSource(Node):
    """Generate one sinusoidal joint position as sensor_msgs/JointState."""

    def __init__(self):
        super().__init__("joint_state_source")

        self.declare_parameter("joint_name", "tutorial_joint")
        self.declare_parameter("amplitude", 1.0)
        self.declare_parameter("frequency", 0.10)
        self.declare_parameter("publish_rate", 20.0)

        self.joint_name = str(self.get_parameter("joint_name").value)
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.frequency = float(self.get_parameter("frequency").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._validate_positive("publish_rate", publish_rate)
        self._validate_positive("amplitude", self.amplitude, allow_zero=True)
        self._validate_positive("frequency", self.frequency, allow_zero=True)

        self.publisher = self.create_publisher(
            JointState,
            "tutorial/joint_states",
            10,
        )
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_state)
        self.add_on_set_parameters_callback(self.update_parameters)

        self.get_logger().info(
            f"Publishing {self.joint_name} at {publish_rate:.1f} Hz; "
            "amplitude and frequency can be changed while running."
        )

    @staticmethod
    def _validate_positive(name, value, allow_zero=False):
        lower_ok = value >= 0.0 if allow_zero else value > 0.0
        if not math.isfinite(value) or not lower_ok:
            comparison = "non-negative" if allow_zero else "positive"
            raise ValueError(f"Parameter '{name}' must be finite and {comparison}.")

    def update_parameters(self, parameters):
        """Accept valid live changes without leaving half-updated state."""
        proposed_amplitude = self.amplitude
        proposed_frequency = self.frequency

        try:
            for parameter in parameters:
                if parameter.name == "amplitude":
                    proposed_amplitude = float(parameter.value)
                    self._validate_positive(
                        "amplitude", proposed_amplitude, allow_zero=True
                    )
                elif parameter.name == "frequency":
                    proposed_frequency = float(parameter.value)
                    self._validate_positive(
                        "frequency", proposed_frequency, allow_zero=True
                    )
                elif parameter.name in {"joint_name", "publish_rate"}:
                    return SetParametersResult(
                        successful=False,
                        reason=f"'{parameter.name}' requires restarting this node.",
                    )
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        self.amplitude = proposed_amplitude
        self.frequency = proposed_frequency
        return SetParametersResult(successful=True)

    def publish_state(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        omega = 2.0 * math.pi * self.frequency

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.joint_name]
        message.position = [self.amplitude * math.sin(omega * elapsed)]
        message.velocity = [self.amplitude * omega * math.cos(omega * elapsed)]
        message.effort = [0.0]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

