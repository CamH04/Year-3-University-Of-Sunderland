#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_callback, 
            10
        )
        self.pub = self.create_publisher(
            Twist, 
            '/diff_drive_controller/cmd_vel_unstamped', 
            10
        )
        self.get_logger().info('CmdVel relay started: /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped')

    def cmd_callback(self, msg: Twist):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

