#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__('cmdvel_to_wheels')
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('wheel_radius', 0.07)
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub_left = self.create_publisher(Float64, 'left_wheel_controller/commands', 10)
        self.pub_right = self.create_publisher(Float64, 'right_wheel_controller/commands', 10)

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z
        vl = (v - omega * self.wheel_base / 2.0) / self.wheel_radius
        vr = (v + omega * self.wheel_base / 2.0) / self.wheel_radius
        self.pub_left.publish(Float64(data=vl))
        self.pub_right.publish(Float64(data=vr))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheels()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

