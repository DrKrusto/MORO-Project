#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class DWAPlanner:
    
    def __init__(self):
        rospy.init_node('dwa_planner')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Robot parameters
        self.max_linear_speed = 0.5
        self.min_linear_speed = 0.0
        self.max_angular_speed = 1.0
        self.min_angular_speed = -1.0

        # DWA parameters
        self.max_acc_linear = 0.1
        self.max_acc_angular = 0.2
        self.dt = 0.1
        self.predict_time = 2.0
        self.simulation_time = 2.0
        self.goal_tolerance = 0.2

        # Robot state
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.laser_data = []

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.robot_pose = np.array([pose.position.x, pose.position.y, 2 * np.arctan2(pose.orientation.z, pose.orientation.w)])

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def plan(self):
        goal = np.array([5.0, 5.0])

        # DWA loop
        while not rospy.is_shutdown():
            linear_speed, angular_speed = self.dwa_control(goal)
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            self.cmd_pub.publish(cmd_vel)
            print(cmd_vel)
            rospy.sleep(self.dt)

    def dwa_control(self, goal):
        # Dynamic Window
        vels = []
        for v in np.arange(self.min_linear_speed, self.max_linear_speed, 0.05):
            for w in np.arange(self.min_angular_speed, self.max_angular_speed, 0.1):
                x, y, theta = self.motion(self.robot_pose, v, w, self.dt)
                if not self.check_collision(x, y):
                    cost = self.calculate_cost([x, y], goal)
                    vels.append([v, w, cost])

        # Find the best velocity
        vels = np.array(vels)
        if len(vels) == 0:
            return 0.0, 0.0  # No valid velocity found

        best_vel = vels[np.argmin(vels[:, 2])]
        return best_vel[0], best_vel[1]

    def motion(self, pose, linear_speed, angular_speed, dt):
        x, y, theta = pose
        x += linear_speed * np.cos(theta) * dt
        y += linear_speed * np.sin(theta) * dt
        theta += angular_speed * dt
        return x, y, theta

    def check_collision(self, x, y):
        # Check collision using laser data
        for r in self.laser_data:
            if r < 0.2:  # Assuming obstacles closer than 0.2 meters are collisions
                return True
        return False

    def calculate_cost(self, current_pose, goal):
        # Simple distance-based cost
        return np.linalg.norm(np.array(current_pose[:2]) - np.array(goal))

if __name__ == '__main__':
    planner = DWAPlanner()
    planner.plan()
