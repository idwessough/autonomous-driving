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
#from nav_msgs.msg import Odometry
from std_msgs.msg import String


class LimoEtat:
    def __init__(self):
        print("Initializing limo_etat node")
        # config rate 
        self.rate = rospy.Rate(rospy.get_param("/rate/nav_limo_projet")) 
        #var pilotage
        self.x = 0.15
        self.z = 0
        self.Twist = Twist()
        # Var state machine
        self.flag_Action = None
        self.ActionEnCours = 0
        # init subscriber/publisher
        self.twist_sub = rospy.Subscriber('/limo_twist', Twist , self.callback_twist)
        self.action_sub = rospy.Subscriber('/limo_action', String, self.callback_action)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Tf buffer 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
    # Callback twist, called by the line_follower and publish continuously to move limo forward via /cmd_vel topic
    def callback_twist(self, data):
        try:
            self.z = data.angular.z
            self.Twist.angular.z = self.z
            self.Twist.linear.x = self.x
            if self.ActionEnCours == 0:
                self.cmd_vel_pub.publish(self.Twist)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            rospy.logerr("twist_callback Error, skipped frame!")
    
    # callback Action, called after sign/person detected to act accordingly depending on situation
    def callback_action(self, data):
        # Récupération de l'action
        Action = data.data
        rospy.loginfo(Action)
        if Action == "STOP":
            self.flag_Action = 1 #Stop
        elif Action == "SLOW":
            self.flag_Action = 2 #Ralentir
        elif Action == "RIGHT":
            self.flag_Action = 3 #VirageDroite
        elif Action == "PASSAGEPIETON":
            self.flag_Action = 4 #PassagePieton
        elif Action == "PIETON":
            self.flag_Action = 5 #Pieton
        elif Action == "REDLIGHT":
            self.flag_Action = 6 #Feu Rouge
        elif Action == "GREENLIGHT":
            self.flag_Action = 7 #Feu vert
        elif Action == "PRIORITY":
            self.flag_Action = 8 #Route prioritaire
        self.state_machine()

    # State Machine, 
    def state_machine(self):
        rospy.loginfo(self.flag_Action)
        if self.flag_Action == 1: #Stop
            msg = Twist()
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('STOP', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 0.6):
                    msg.angular.z = 0
                    msg.linear.x = 0
                    self.ActionEnCours = 1
                    self.cmd_vel_pub.publish(msg)
                    rospy.sleep(3)
                    self.ActionEnCours = 0
                    break

        elif self.flag_Action == 2: #Ralentir
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('SLOW', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 0.6):
                    self.x = 0.07
                    rospy.sleep(3)
                    self.x = 0.15
                    break

        elif self.flag_Action == 3: #VirageDroite
            
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('RIGHT', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 0.6):
                    self.ActionEnCours = 1
                    self.timer = rospy.Timer(rospy.Duration(1.0/100.0), self.turn_right)
                    rospy.sleep(7)
                    self.timer.shutdown()
                    self.ActionEnCours = 0
                    break

        elif self.flag_Action == 4: #PassagePieton
            msg = Twist()
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('PASSAGEPIETON', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 0.55):
                    msg.angular.z = 0
                    msg.linear.x = 0
                    self.ActionEnCours = 1
                    self.cmd_vel_pub.publish(msg)
                    rospy.sleep(5)
                    self.ActionEnCours = 0
                    break

        elif self.flag_Action == 5: #Pieton
            msg = Twist()
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('PIETON', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 0.6):
                    msg.angular.z = 0
                    msg.linear.x = 0
                    self.ActionEnCours = 1  
                    self.cmd_vel_pub.publish(msg)
                    rospy.sleep(5)
                    self.ActionEnCours = 0
                    break

        elif self.flag_Action == 6: #feu rouge
            msg = Twist()
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('REDLIGHT', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 1):
                    msg.angular.z = 0
                    msg.linear.x = 0
                    self.ActionEnCours = 1
                    self.cmd_vel_pub.publish(msg)
                    while(self.flag_Action != 7):
                        pass
                    self.ActionEnCours = 0
                    break

        elif self.flag_Action == 8: #Route prioritaire
            while (1):
                try:
                    trans = self.tfBuffer.lookup_transform('PRIORITY', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                seuil = np.sqrt(np.power(trans.transform.translation.y, 2)+np.power(trans.transform.translation.x, 2))
                rospy.loginfo(seuil)
                if( seuil < 1):
                    self.x = 0.18
                    break

    def turn_right(self, timer):
        msg = Twist()
        msg.angular.z = -0.3
        msg.linear.x = 0.15
        self.cmd_vel_pub.publish(msg)
        self.rate.sleep()

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

