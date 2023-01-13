#!/usr/bin/env python

import rospy
import cv2
import sys
import traceback
import numpy 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class LineDetector:
    def __init__(self):
        print("Initializing line detector node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback_raw)
        self.image_pub = rospy.Publisher("processed_image", Image, queue_size=40)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.line_offset = 0
        self.Twist = Twist()

    def image_callback_raw(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.process_image(cv_image)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("CvBridge Error, skipped image frame!")

    def process_image(self, cv_image):
        global perr, ptime, serr, dt
    
        #transformation
        img = cv2.resize(cv_image,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
        #print img.shape
        rows, cols, ch = img.shape
        pts1 = numpy.float32([[90,122],[313,122],[35,242],[385,242]])
        pts2 = numpy.float32([[0,0],[400,0],[0,400],[400,400]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        img_size = (img.shape[1], img.shape[0])
        image = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 10,  10,  10])
        upper_yellow = numpy.array([255, 255, 250])

        lower_white = numpy.array([100,100,200], dtype= "uint8")
        upper_white = numpy.array([255,255,255], dtype= "uint8")

        #threshold to get only white
        maskw = cv2.inRange(image, lower_white, upper_white)
        masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #remove pixels not in this range
        mask_yw = cv2.bitwise_or(maskw, masky)    
        rgb_yw = cv2.bitwise_and(image, image, mask = mask_yw).astype(numpy.uint8)

        rgb_yw = cv2.cvtColor(rgb_yw, cv2.COLOR_RGB2GRAY)
    
        #filter mask
        kernel = numpy.ones((7,7), numpy.uint8)
        opening = cv2.morphologyEx(rgb_yw, cv2.MORPH_OPEN, kernel)
        rgb_yw2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)		
        
        h, w= rgb_yw2.shape
        search_top = 7*h/8+20
        search_bot = 7*h/8 + 800 +20
        rgb_yw2[0:search_top, 0:w] = 0
        rgb_yw2[search_bot:h, 0:w] = 0
        M = cv2.moments(rgb_yw2)

        # calculate x,y coordinate of center
        cX = 128
        cY = 128
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            cx = cX - 110 
            if cX <= 2*h/8:
                cx = cX +(h/2)
            
            err = cx - w/2
            self.Twist.linear.x = 0.1
            self.Twist.angular.z =  (-float(err) / 100)*2.5 + ((err - perr)/(rospy.get_time() - ptime))*1/50/100
            serr = err + serr
            perr = err
            ptime = rospy.get_time()
            self.cmd_vel_pub.publish(self.Twist)

        # Change from grayscale
        #rgb_yw2 = cv2.cvtColor(rgb_yw2, cv2.COLOR_BGR2HSV)

        #Center of bottom section
        rgb_yw2 = cv2.circle(rgb_yw2, (cX, cY), 5, (255, 255, 0), -1)
        rgb_yw2 = cv2.circle(rgb_yw2, (cx, cY), 5, (255,255,0),-1)

        # Output the processed message
        image_message = self.bridge.cv2_to_imgmsg(rgb_yw2, "passthrough")
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
