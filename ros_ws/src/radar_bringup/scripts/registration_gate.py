#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from radar_interfaces.msg import RegistrationStatus


class RegistrationGate(Node):
    def __init__(self):
        super().__init__("registration_gate")
        self.exit_code = None
        self.subscription = self.create_subscription(
            RegistrationStatus,
            "/localization/registration_status",
            self.on_status,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def on_status(self, status):
        if status.state == RegistrationStatus.LOCKED:
            self.exit_code = 0
        elif status.state == RegistrationStatus.FAILED:
            print(status.reason or "registration failed", file=sys.stderr)
            self.exit_code = 1


def main():
    rclpy.init()
    gate = RegistrationGate()
    try:
        while gate.exit_code is None:
            rclpy.spin_once(gate)
        return gate.exit_code
    finally:
        gate.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
