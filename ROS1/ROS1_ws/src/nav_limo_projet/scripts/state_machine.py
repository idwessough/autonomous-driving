#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf
import sys
import traceback
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sign_tf2_broadcaster
#from nav_msgs.msg import Odometry
from std_msgs.msg import String


class LimoEtat:
    def __init__(self):
        print("Initializing limo_etat node")
        # config rate 
        self.rate = rospy.Rate(rospy.get_param("/rate/nav_limo_projet")) 
        # init subscriber/publisher
        self.twist_sub = rospy.Subscriber('/limo_twist', Twist , self.callback_twist)
        self.action_sub = rospy.Subscriber('/limo_action', String, self.callback_action)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Tf buffer 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #var pilotage
        self.x = 0.15
        self.z = 0
        self.Twist = Twist()
        # test with transform
        sign_tf2_broadcaster.handle_sign_pose(3, 3, "stop")
        # Var state machine
        self.Action = None
        self.flag_Action = None

    def callback_twist(self, data):
        try:
            self.z = data.data.angular.z
            self.Twist.angular.z = self.z
            self.Twist.linear.x = self.x
            self.cmd_vel_pub.publish(self.Twist)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("twist_callback Error, skipped frame!")
    
    def callback_action(self, data):
        # Récupération de l'action
        self.Action = data.data
        if self.Action == "STOP":
            self.flag_Action = 1 #Stop
        elif self.Action == "RALENTIR":
            self.flag_Action = 2 #Ralentir
        elif self.Action == "VIRAGEDROITE":
            self.flag_Action = 3 #VirageDroite
        elif self.Action == "PASSAGEPIETON":
            self.flag_Action = 4 #PassagePieton
        elif self.Action == "PIETON":
            self.flag_Action = 5 #Pieton
        self.state_machine()

    # State Machine
    def state_machine(self):
        if self.flag_Action == 1: #Stop
            msg = Twist()
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('Stop', 'base_link', rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                if(np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2)) < 10):
                    msg.angular.z = 0
                    msg.linear.x = 0
                    self.x = 0
                    self.cmd_vel_pub.publish(msg)
                    self.flag_Action = None
                    ((self.rate)*10).sleep()
                    self.x = 0.15
                    break

        # elif self.flag_Action == 2: #Ralentir
        #     msg = Twist()
        #     while (1):
        #         try:
        #             trans = self.tfBuffer.lookup_transform('Stop', 'base_link', rospy.Time(0))
        #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             pass
        #         if(np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2)) < 10):
        #             self.x = 0.07
        #             self.flag_Action = None
        #             ((self.rate)*10).sleep()
        #             self.x = 0.15
        #             break

        # elif self.flag_Action == 3: #VirageDroite
        #     msg = Twist()
        #     while (1):
        #         try:
        #             trans = self.tfBuffer.lookup_transform('VirageDroite', 'base_link', rospy.Time(0))
        #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             pass
        #         if(np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2)) < 5):
        #             msg.angular.z = 0.5
        #             msg.linear.x = 0.15
        #             self.cmd_vel_pub.publish(msg)
        #             self.rate.sleep()
        #             msg.angular.z = 0.3
        #             msg.linear.x = 0.15
        #             self.cmd_vel_pub.publish(msg)
        #             self.rate.sleep()
        #             msg.angular.z = 0.1
        #             msg.linear.x = 0.15
        #             self.cmd_vel_pub.publish(msg)
        #             self.rate.sleep()
        #             msg.angular.z = 0
        #             msg.linear.x = 0.15
        #             self.cmd_vel_pub.publish(msg)
        #             self.rate.sleep()
        #             self.flag_Action = None
        #             break

        # elif self.flag_Action == 4: #PassagePieton
        #     msg = Twist()
        #     while (1):
        #         try:
        #             trans = self.tfBuffer.lookup_transform('PassagePieton', 'base_link', rospy.Time(0))
        #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             pass
        #         if(np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2)) < 10):
        #             msg.angular.z = 0
        #             msg.linear.x = 0
        #             self.x = 0 
        #             self.cmd_vel_pub.publish(msg)
        #             self.flag_Action = None
        #             ((self.rate)*10).sleep()
        #             self.x = 0.15
        #             break

        # elif self.flag_Action == 5: #Pieton
        #     msg = Twist()
        #     while (1):
        #         try:
        #             trans = self.tfBuffer.lookup_transform('Pieton', 'base_link', rospy.Time(0))
        #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             pass
        #         if(np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2)) < 10):
        #             msg.angular.z = 0
        #             msg.linear.x = 0
        #             self.x = 0  
        #             self.cmd_vel_pub.publish(msg)
        #             self.flag_Action = None
        #             ((self.rate)*10).sleep()
        #             self.x = 0.15
        #             break

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

def main(args):
    rospy.init_node('limo_state_machine')
    limo_etat = LimoEtat()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ =='__main__':
    main(sys.argv)

