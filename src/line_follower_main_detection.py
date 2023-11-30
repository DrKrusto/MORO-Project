#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32

class LineCenterMainDetection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.line_detection_pub = rospy.Publisher('line_detection', Image, queue_size=1)
        self.line_position_pub = rospy.Publisher('line_position', Int32, queue_size=1)
        self.line_x_position_pub = rospy.Publisher('line_x_position', Float32, queue_size=1)
        self.center_portion = 0
        self.image_sub = rospy.Subscriber('/camera/image',
                                          Image, self.image_callback)

    def image_callback(self, msg):
        # Converts the image to a numpy array
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Defines the range of yellow color
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Creates a mask with the yellow color
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Defines the search area
        h, w, _ = image.shape
        search_top = 3 * h // 4
        search_bot = 3 * h // 4 + 20

        # Creates a mask for the search area
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # Calculates the moments of the mask
        moments = cv2.moments(mask)

        if moments['m00'] > 0:
            # Calculates the center of the line
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

            # Calculates the width of the line
            width = np.count_nonzero(mask[cy, :])

            # Draws a circle on the center
            cv2.circle(image, (cx, cy), 3, (0, 0, 255), -1)

            # Create a colored bounding box around the center position of the line
            cv2.rectangle(image, (cx - width // 2, cy - 10),
                          (cx + width // 2, cy + 10), (0, 0, 255), 2)
            
            # Create text on top of the bounding box
            cv2.putText(image, "Line Detected", (cx - width // 2, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Calculates on which portion of the image the line is
            new_position = -1 if cx < w/3 else 1 if cx < 2*w/3 else 0
        
            # We avoid publishing the same information over and over again
            if (self.center_portion != new_position):
                self.center_portion = new_position
                self.line_position_pub.publish(self.center_portion)

            # Publish the x position of the line
            self.line_x_position_pub.publish(cx)

        # Shows the image
        cv2.imshow("output", image)
        cv2.waitKey(3)

        # Publishes the image
        self.line_detection_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))

if __name__ == "__main__":
    rospy.init_node('line_detection')
    rospy.loginfo("Line detection node started")
    line_center_detector = LineCenterMainDetection()
    rospy.spin()
