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
    
    def go_to(self, goal_point:Point) -> None:
        speed : Twist = Twist()
        rho = float("inf")

        while rho > 0.2:
            delta_x = goal_point.x - self.x
            delta_y = goal_point.y - self.y
            rho = sqrt(delta_x**2 + delta_y**2)
            angle_to_goal = atan2(delta_y, delta_x)

            if abs(angle_to_goal - self.theta) > 0.1: # Lets the robot rotate
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else: # Lets the robot drive forward
                speed.linear.x = 0.22
                speed.angular.z = 0.0
            self.pub.publish(speed)
            rospy.sleep(0.01)

    def stop_robot(self) -> None: # Function to stop the robot
        speed : Twist = Twist()
        
        speed.linear.x = 0.0
        speed.angular.z = 0.0

        self.pub.publish(speed)


if __name__ == '__main__':
    rospy.init_node('motion_node')
    simple_pose_controller = SimplePoseController()

    rospy.spin()