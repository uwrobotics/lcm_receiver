#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lcm
import os
import sys
sys.path.append(os.path.join(os.getcwd(), 'lcm_receiver', 'lcm_receiver'))
from geometry import twist_t


class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        lc = lcm.LCM()
        self.publisher_ = self.create_publisher(Twist, 'joystick_input', 10)
        subscription = lc.subscribe("control", self.my_handler)
        self.get_logger().info('Twist publisher has been started.')
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def my_handler(self, channel, data):
        msg = twist_t.decode(data)
        ros_msg = Twist()
        ros_msg.linear.x = list(msg.linear)[0]
        ros_msg.angular.z = list(msg.angular)[2]
        print(ros_msg)
        self.publisher_.publish(ros_msg)
        self.get_logger().info(f'Publishing: linear.x={ros_msg.linear.x}, angular.z={ros_msg.angular.z}')
        

        
def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
