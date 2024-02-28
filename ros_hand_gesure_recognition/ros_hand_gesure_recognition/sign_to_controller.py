#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class GestureController:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('hand_sign_control')
        
        # Subscriber for subscribing the hand signs
        self.gesture_subscriber = self.node.create_subscription(
            String, 
            self.callback, 
            '/hand_sign_recognition/publish_gesture_topic', 
            10
        )

        # Publisher for publishing velocities 
        self.vel_publisher = self.node.create_publisher(Twist, '/robot_diff_drive_controller/cmd_vel', 10)

        # Velocity message
        self.vel_msg = Twist()
        
        # Velocity increments
        self.linear_vel = 0.01  # [m/s]
        self.angular_vel = 0.1  # [rad/s]

    def callback(self, gesture):
        if gesture.data == "Forward":
            self.vel_msg.linear.x += self.linear_vel
            self.vel_msg.angular.z = 0.0
        elif gesture.data == "Backward":
            self.vel_msg.linear.x -= self.linear_vel
            self.vel_msg.angular.z = 0.0
        elif gesture.data == "Turn Right":
            self.vel_msg.angular.z -= self.angular_vel
        elif gesture.data == "Turn Left":
            self.vel_msg.angular.z += self.angular_vel
        elif gesture.data == "Stop" or gesture.data == "NONE":
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        
        self.vel_publisher.publish(self.vel_msg)
        print(f"{gesture.data} Linear: {round(self.vel_msg.linear.x, 6)} m/s, Angular: {round(self.vel_msg.angular.z, 6)}")

if __name__ == "__main__":
    try:
        ges2control = GestureController()
        rclpy.spin(ges2control.node)
    except KeyboardInterrupt:
        pass
    finally:
        ges2control.node.destroy_node()
        rclpy.shutdown()

