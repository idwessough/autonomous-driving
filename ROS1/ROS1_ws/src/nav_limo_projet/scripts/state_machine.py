#!/usr/bin/env python

import rospy
import tf2_ros
import tf
import sys
import traceback
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from std_msgs.msg import String


class LimoEtat:
    def __init__(self):
        print("Initializing limo_etat node")
        # config rate 
        self.rate = rospy.Rate(rospy.get_param("/rate/nav_limo_projet")) 
        # init subscriber/publisher
        self.rospy.Subscriber('/limo_twist', Twist , self.callback_twist)
        self.rospy.Subscriber('/limo_action', String, self.callback_action)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Tf buffer 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #var pilotage
        self.x = 0
        self.z = 0
        self.Twist = Twist()
        # Var state machine
        self.Action = None
        self.flag_Action = None

    def action_vitesseNormale(self):
        self.x = 0.15

    def action_stop(self):
        self.x = 0

    def action_ralentir(self):
        self.x = 0.07
    
    def action_virageDroite(self):
        self.z = 0.5
        self.action_vitesseNormale()
    

    def callback_twist(self, data):
        try:
            self.z = data.data.angular.z
            self.Twist.angular.z = self.z
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

    # State Machine
    def state_machine(self, Action_flag):
        if self.flag_Action == 1: #Stop
            try:
                trans_stop = self.tfBuffer.lookup_transform('Stop', 'base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            msg = Twist()

            msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        elif self.flag_Action == 2: #Ralentir
            
        elif self.flag_Action == 3: #VirageDroite

        elif self.flag_Action == 4: #PassagePieton

        elif self.flag_Action == 5: #Pieton
            
        else:
            self.action_vitesseNormale()

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

