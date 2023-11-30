#!/usr/bin/env python3

# Author: Markus Luftensteiner

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from math import atan2, sqrt, hypot

class SimplePoseController:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/line_position', String, self.line_position_callback)
        self.sub2 = rospy.Subscriber('/line_x_position', Float32, self.line_x_position_callback)
        self.rate = rospy.Rate(1)

    # Controls the motion of the robot -> forward, backward, clockwise, counterclockwise, stop
    def line_position_callback(self, msg:String):
        command = msg.data
        speed : Twist = Twist()

        if command == "centre":
            speed.linear.x = 0.22
        elif command == "right":
            speed.angular.z = 0.3
        elif command == "left":
            speed.angular.z = -0.3
        elif command == "stop":
            speed.linear.x = 0
            speed.angular.z = 0
        else: # ignore all other values
            pass
        
        self.pub.publish(speed)
    
    def line_x_position_callback(self, action:Float32) -> None:
        action_val = float(action.data)
        speed : Twist = Twist()

        # Message is centered on the image, negative = left, positive = right
        if action_val < 0: # Rotate left
            speed.linear.x = 0.0
            speed.angular.z = -0.3
        elif action_val > 0: # Rotate right
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif action_val == 0: # Move forward
            speed.linear.x = 0.22
            speed.angular.z = 0
        else: # In all other cases stop the robot
            speed.linear.x = 0.0
            speed.angular.z = 0.0

        self.pub.publish(speed)
        rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('motion_node')
    simple_pose_controller = SimplePoseController()

    rospy.spin()
