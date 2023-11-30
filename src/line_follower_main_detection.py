#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
# from std_msgs.msg import Int

class LineCenterMainDetection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # self.line_detection_pub = rospy.Publisher('line_detection', Image, queue_size=1)
        # self.line_position_pub = rospy.Publisher('line_position', Int, queue_size=1)
        # self.line_x_position_pub = rospy.Publisher('line_x_position', Int, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/image',
                                          Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, _ = image.shape
        search_top = 3 * h // 4
        search_bot = 3 * h // 4 + 20

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        moments = cv2.moments(mask)

        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

            cv2.circle(image, (cx, cy), 3, (0, 0, 255), -1)

        cv2.imshow("output", image)
        cv2.waitKey(3)

        # self.line_detection_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='mono8'))

if __name__ == "__main__":
    rospy.init_node('line_detection')
    rospy.loginfo("Line detection node started")
    line_center_detector = LineCenterMainDetection()
    rospy.spin()
