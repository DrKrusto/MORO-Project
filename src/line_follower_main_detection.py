#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class LineCenterMainDetection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher('/camera/image', Image, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/image',
                                          Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert the image to grayscale
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Publish to camera/image to test the grayscale image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(grayscale_image, encoding='mono8'))

if __name__ == "__main__":
    rospy.init_node('line_detection')
    rospy.loginfo("Line detection node started")
    line_center_detector = LineCenterMainDetection()
    rospy.spin()
