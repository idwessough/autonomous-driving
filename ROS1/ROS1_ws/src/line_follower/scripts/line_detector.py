#!/usr/bin/env python

import rospy
import cv2
import sys
import traceback
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

class LineDetector:
    def __init__(self):
        print("Initializing line detector node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.image_sub_comp = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, self.image_callback_compressed)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback_raw)
        self.image_pub = rospy.Publisher("processed_image", Image, queue_size=40)
        self.bridge = CvBridge()
        self.line_offset = 0

    def image_callback_raw(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.process_image(cv_image)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("CvBridge Error, skipped image frame!")

    def image_callback_compressed(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(image_np)
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("skipped processed image frame!")

    def process_image(self, cv_image):
        # Downscale to 256x256
        cv_image = cv2.resize(cv_image, (256, 256), interpolation = cv2.INTER_AREA)

        # Change to grayscale and blur a bit
        k_width = 5
        k_height = 5
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.GaussianBlur(cv_image, (k_width, k_height), 0)

        # Threshold the image
        (T, threshold_image) = cv2.threshold(cv_image, 210, 255, cv2.THRESH_BINARY)
        #threshold_image = cv2.adaptiveThreshold(cv_image,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)

        # Overlay black box on top of image
        cv2.rectangle(threshold_image, (0, 0), (256, 150), (0,0,0), -1)

        #detect line
        threshold1 = 85
        threshold2 = 85
        threshold_image = cv2.Canny(threshold_image, threshold1, threshold2)

        #line detection
        minLineLength = 6
        maxLineGap = 20
        max_slider = 10 
        lines = cv2.HoughLinesP(threshold_image,1,np.pi/180,max_slider,minLineLength,maxLineGap)

        # # Get center of the thresholded image
        # M = cv2.moments(threshold_image)

        # # calculate x,y coordinate of center
        # cX = 128
        # cY = 128
        # if M["m00"] != 0:
        #     cX = int(M["m10"] / M["m00"])
        #     cY = int(M["m01"] / M["m00"])

        # # Compute an offset in [-1, 1] coordinates to convert to steering
        # self.line_offset = (cX - 128.0) / 128.0

        # Change from grayscale
        threshold_image = cv2.cvtColor(threshold_image, cv2.COLOR_GRAY2RGB)

        # Overlay black box on top of image
        cv2.rectangle(threshold_image, (0, 200), (256, 200), (255, 255, 255), -1)
        
        #Center of bottom section
        #threshold_image = cv2.circle(threshold_image, (cX, cY), 5, (0, 255, 0), -1)

        #visualize lines detected by hough tf
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                cv2.line(threshold_image,(x1,y1),(x2,y2),(255,180,0),1)

        # The centerline of the image
        threshold_image = cv2.line(threshold_image,(128,0),(128,256),(0,0,255),1)

        # Bottom quadrant that is used to signal which direction to go
        threshold_image = cv2.line(threshold_image,(0,200),(256,200),(0,0,255),1)

        # visualize line offset
        threshold_image = cv2.line(threshold_image,(int(128 + (self.line_offset * 128)),0),(int(128 + (self.line_offset * 128)),256),(255,0,255),1)

        # Output the processed message
        image_message = self.bridge.cv2_to_imgmsg(threshold_image, "passthrough")
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
