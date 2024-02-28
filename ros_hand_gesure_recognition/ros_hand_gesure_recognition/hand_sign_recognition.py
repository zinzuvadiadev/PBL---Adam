#!/usr/bin/python3

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from gesture_recognition import GestureRecognition
from cv_bridge import CvBridge
from cvfpscalc import CvFpsCalc
import cv2 as cv

class HandSignRecognition:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('hand_sign_recognition', anonymous=True)

        image_topic = self.node.get_parameter("hand_sign_recognition/subscribe_image_topic").get_parameter_value().string_value
        self.image_subscriber = self.node.create_subscription(Image, image_topic, self.callback, 10)

        gesture_topic = self.node.get_parameter("hand_sign_recognition/publish_gesture_topic").get_parameter_value().string_value
        self.gesture_publisher = self.node.create_publisher(String, gesture_topic, 10)

        self.gesture_detector = GestureRecognition(
            self.node.get_parameter("hand_sign_recognition/keypoint_classifier_label").get_parameter_value().string_value,
            self.node.get_parameter("hand_sign_recognition/keypoint_classifier_model").get_parameter_value().string_value
        )
        
        self.bridge = CvBridge()
        self.cv_fps_calc = CvFpsCalc(buffer_len=10)
    def callback(self, image_msg):        
    	try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg)
            debug_image, gesture = self.gesture_detector.recognize(cv_image)
            self.gesture_publisher.publish(gesture)
            if self.node.get_parameter("hand_sign_recognition/show_image").get_parameter_value().bool_value:
                fps = self.cv_fps_calc.get()
                debug_image = self.gesture_detector.draw_fps_info(debug_image, fps)
                cv.imshow('ROS Gesture Recognition', debug_image)
                cv.waitKey(10)  # wait for 10 milliseconds
        except CvBridgeError as error:
            print(error)

if __name__ == "__main__":
    try:
        hand_sign = HandSignRecognition()
        rclpy.spin(hand_sign.node)
        hand_sign.node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        cv.destroyAllWindows()

