#!/usr/bin/env python

import rospy
import sys
import traceback
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class steering:
    def __init__(self):
        print("Initializing steering node")
        # read rate config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.offset_sub = rospy.Subscriber("/lane_detection/off_center", Float32, self.offset_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.line_offset = 0.
        self.Twist = Twist()
        self.perr = 0
        self.ptime = 0
        self.serr = 0

    def offset_callback(self, data):
        try:
            self.line_offset = data.data
            rospy.loginfo(self.line_offset)
            self.ackermann(self.line_offset)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("Ackermann Error, skipped calculations")

    def ackermann(self, line_offset):

        err = line_offset
        err = err-0.70
        self.Twist.linear.x = 0.15
        self.Twist.angular.z = float(err)

        # Output the twist angle
        self.ptime = rospy.get_time()
        self.cmd_vel_pub.publish(self.Twist)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


def main(args):
    rospy.init_node('steering')
    steering_syst = steering()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ =='__main__':
    main(sys.argv)
