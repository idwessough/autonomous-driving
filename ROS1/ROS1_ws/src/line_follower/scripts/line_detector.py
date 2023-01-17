#!/usr/bin/env python

import rospy
import cv2
import sys
import traceback
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

max_inner_angle = 0.48869;  # 28 degree
track = 0.172;              # m (left right wheel distance)
wheelbase = 0.2;            # m (front rear wheel distance)

class LineDetector:
    def __init__(self):
        print("Initializing line detector node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback_raw)
        self.image_pub = rospy.Publisher("processed_image", Image, queue_size=40)
        self.cmd_vel_pub = rospy.Publisher('/limo_twist', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.line_offset = 0
        self.Twist = Twist()
        self.perr = 0
        self.ptime = 0
        self.serr = 0

    def image_callback_raw(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.process_image(cv_image)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("CvBridge Error, skipped image frame!")

    def process_image(self, cv_image):
        # Downscale to 256x256
        cv_image = cv2.resize(cv_image, (256, 256), interpolation = cv2.INTER_AREA)

        ### histogram equalization ###

        # Convert the image from BGR to YCrCb color space
        hist_equalized_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCrCb)
        # Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a vector
        Y, CR, CB = cv2.split(hist_equalized_image)
        # Equalize the histogram of only the Y channel
        Y = cv2.equalizeHist(Y, Y)
        # Merge 3 channels in the vector to form the color image in YCrCB color space.
        hist_equalized_image = cv2.merge([Y, CR, CB])
        #Convert the histogram equalized image from YCrCb to BGR color space again
        hist_equalized_image = cv2.cvtColor(hist_equalized_image, cv2.COLOR_YCrCb2BGR)

        ### end histrogram equlization ###

        # Change color and blur a bit
        cv_image = cv2.cvtColor(hist_equalized_image, cv2.COLOR_BGR2HSV)
        #cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
        
        #white
        low = np.array([ 80, 0, 240])
        high = np.array([ 255, 20, 255])

        mask = cv2.inRange(cv_image, low, high)

        h, w, d = cv_image.shape

        search_top = h/2+62
        search_bot = h
        search_left = w/2
        search_right = w
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right:w] = 0

        #moment des lignes
        target = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        M = cv2.moments(mask)

        # calculate x,y coordinate of center
        cX = w/2
        cY = h/2
        cx = 0
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            cx = cX - 90

            err = w/2 - cx 
            self.Twist.linear.x = 0.15
            self.Twist.angular.z = float(err) / 100 #(float(err) / 500)*2.5 + ((err - self.perr)/(rospy.get_time() - self.ptime))*1/50/100
            self.serr = err + self.serr
            self.perr = err
            self.ptime = rospy.get_time()
            self.cmd_vel_pub.publish(self.Twist)

        # Change from grayscale
        target = cv2.cvtColor(target, cv2.COLOR_BGR2HSV)

        #Center of line right
        target = cv2.circle(target, (cX, cY), 5, (0, 255, 0), -1)
        target = cv2.circle(target, (cx, cY), 5, (0, 255, 0), -1)

        # Output the processed message
        image_message = self.bridge.cv2_to_imgmsg(target, "passthrough")
        self.image_pub.publish(image_message)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


def main(args):
    rospy.init_node('line_detector')
    line_detect = LineDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ =='__main__':
    main(sys.argv)
