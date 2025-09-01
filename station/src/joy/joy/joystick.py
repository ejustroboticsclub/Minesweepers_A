#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from inputs import get_gamepad


class JoyControl(Node):
    def __init__(self):
        super().__init__("control_node")
        self.pub_velocity = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_gripper = self.create_publisher(Int16, "/controller", 10)
        self.velocity = Twist()
        self.timer = self.create_timer(0.05, self.gamepad_loop)  # 20Hz

    def gamepad_loop(self):
        events = get_gamepad()
        for event in events:
            if event.code == "ABS_Y":
                self.velocity.linear.x = self.map_value(event.state)
            if event.code == "ABS_X":
                self.velocity.angular.z = -self.map_value(event.state)
            if event.code == "BTN_PINKIE" and event.state == 1:
                self.get_logger().info("SpeedUp")
                self.pub_gripper.publish(Int16(data=5))
            if event.code == "BTN_BASE2" and event.state == 1:
                self.get_logger().info("SpeedDown")
                self.pub_gripper.publish(Int16(data=6))
            if event.code == "BTN_TRIGGER":
                if event.state == 1:
                    self.get_logger().info("ArmUp")
                    self.pub_gripper.publish(Int16(data=7))
                else:
                    self.get_logger().info("Stop lifting")
                    self.pub_gripper.publish(Int16(data=9))
            if event.code == "BTN_THUMB2":
                if event.state == 1:
                    self.get_logger().info("ArmDown")
                    self.pub_gripper.publish(Int16(data=8))
                else:
                    self.get_logger().info("Stop lifting")
                    self.pub_gripper.publish(Int16(data=9))
            if event.code == "BTN_THUMB" and event.state == 1:
                self.get_logger().info("ReleaseMine")
                self.pub_gripper.publish(Int16(data=10))
            if event.code == "BTN_TOP" and event.state == 1:
                self.get_logger().info("HoldMine")
                self.pub_gripper.publish(Int16(data=11))

            self.get_logger().info(
                f"Velocity: linear={self.velocity.linear.x:.2f}, angular={self.velocity.angular.z:.2f}"
            )
            self.pub_velocity.publish(self.velocity)

    @staticmethod
    def map_value(x, in_min=0, in_max=255, out_min=1, out_max=-1):
        if 120 <= x <= 135:
            return 0
        return float(out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min))


def main(args=None):
    rclpy.init(args=args)
    node = JoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
