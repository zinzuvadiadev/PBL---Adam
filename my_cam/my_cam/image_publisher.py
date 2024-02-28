#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class MyCamPublisher(Node):
    def __init__(self):
        super().__init__('my_cam')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_image)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture("/dev/video0")

    def publish_image(self):
        ret, img = self.capture.read()
        if not ret:
            self.get_logger().error('Could not grab a frame!')
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.publisher_.publish(img_msg)
        except CvBridgeError as error:
            self.get_logger().error(str(error))

def main(args=None):
    rclpy.init(args=args)

    my_cam_publisher = MyCamPublisher()

    try:
        rclpy.spin(my_cam_publisher)
    except KeyboardInterrupt:
        pass

    my_cam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
