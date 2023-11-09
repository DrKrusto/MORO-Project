#!/usr/bin/env python3

import rospy
from geometry_msgs import Twist
from nav_msgs import Odometry
from std_msgs import String

class SimplePoseController:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/motion', String)
        self.rate = rospy.Rate(1)

    # Controls the motion of the robot -> forward, backward, clockwise, counterclockwise, stop
    def motion_callback(self, msg:String):
        command = msg.data
        speed : Twist = Twist()

        if command == "forward":
            speed.linear.x = 0.22
        elif command == "backward":
            speed.linear.x = -0.22
        elif command == "clockwise":
            speed.angular.z = 0.3
        elif command == "counterclockwise":
            speed.angular.z = -0.3
        elif command == "stop":
            speed.linear.x = 0
            speed.angular.z = 0

        self.pub.publish(speed)

if __name__ == '__main__':
    rospy.init_node('motion_node')
    simple_pose_controller = SimplePoseController()

    rospy.spin()