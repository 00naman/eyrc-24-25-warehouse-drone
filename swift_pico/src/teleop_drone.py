#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from swift_msgs.msg import SwiftMsgs
import cv2


class TrackbarPublisher(Node):
    def __init__(self):
        super().__init__('trackbar_publisher')
        
        # Publishers for the slider values
        self.slider1_pub = self.create_publisher(Int32, 'throttle', 10)
        self.slider2_pub = self.create_publisher(Int32, 'roll', 10)
        self.slider3_pub = self.create_publisher(Int32, 'pitch', 10)

        self.rc_roll = 1000
        self.rc_pitch = 1000
        self.rc_throttle = 1000
        self.drone_pub = self.create_publisher(SwiftMsgs, '/drone_command', 1)

        # Create an OpenCV window with trackbars
        cv2.namedWindow('Trackbars')
        
        # Create 3 trackbars for the sliders
        cv2.createTrackbar('throttle', 'Trackbars', 1000, 2000, self.slider1_callback)
        cv2.createTrackbar('roll', 'Trackbars', 1000, 2000, self.slider2_callback)
        cv2.createTrackbar('pitch', 'Trackbars', 1000, 2000, self.slider3_callback)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def slider1_callback(self, value):
        msg = Int32()
        msg.data = value
        self.rc_throttle = value
        # self.slider1_pub.publish(msg)

    def slider2_callback(self, value):
        msg = Int32()
        msg.data = value
        self.rc_roll = value
        # self.slider2_pub.publish(msg)

    def slider3_callback(self, value):
        msg = Int32()
        msg.data = value
        self.rc_pitch = value
        # self.slider3_pub.publish(msg)

    def timer_callback(self):
        cv2.waitKey(1)

        cmd = SwiftMsgs()
        cmd.rc_throttle = self.rc_throttle
        cmd.rc_roll = self.rc_roll
        cmd.rc_pitch = self.rc_pitch

        self.drone_pub.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node = TrackbarPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
