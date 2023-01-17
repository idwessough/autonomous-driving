#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class LimoEtat:
    def __init__(self):
        print("Initializing limo_etat node")
        # config
        self.rate = rospy.Rate(rospy.get_param("/rate/lineDetector")) 
        self.rospy.Subscriber('/limo_angle', Float32 , self.callback_angle)
        self.rospy.Subscriber('/limo_action', String, self.callback_action)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #var pilotage
        self.x = 0
        self.z = 0
        self.Twist = Twist()
        self.Action


    def action_vitesseNormale(self):
        self.x = 0.15

    def action_stop(self):
        self.x = 0

    def action_ralentir(self):
        self.x = 0.07
    
    def action_virageDroite(self):
        self.z = 0.5
        self.action_vitesseNormale()

    

       
    def callback_angle(self, data):
       
       # Récupération de l'angle
        self.z = data.data

        # Mise à jour de l'angle
        self.Twist.angular.z = self.z

        # Publish
        self.cmd_vel_pub.publish(self.Twist)

    
    def callback_action(self, data):

        # Récupération de l'action
        self.Action = data.data
        
        if self.Action == "STOP":
            self.action_stop()

        elif self.Action == "RALENTIR":
            self.action_ralentir()

        elif self.Action == "VIRAGEDROITE":
            self.action_virageDroite()

        else:
            self.action_vitesseNormale()
            
       
        # Mise à jour de x
        self.Twist.linear.x = self.x

        # Publish
        self.cmd_vel_pub.publish(self.Twist)


def main(args):
    rospy.init_node('limo_state_machine')
    limo_etat = LimoEtat()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ =='__main__':
    main(sys.argv)

