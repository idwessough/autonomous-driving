import rospy
import cv2
import sys
import traceback
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from std_msgs.msg import String

class LimoEtat:
    def __init__(self):
        print("Initializing line detector node")
        # read rate config
        self.rospy.Subscriber('/limo_angle', Float32 , self.callback)
        self.rospy.Subscriber('/limo_action', String , self.callback)
        self.rospy.Subscriber('/limo_x', Float32 , self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.limo_x = rospy.Publisher('/limo_x', Float32, queue_size=1)
        self.Twist = Twist()


    def action_limo(self, action):
        if action == "STOP":
            self.limo_x.publish(0)
        elif action == "RALENTIR":
            self.limo_x.publish(0.07)
        else:
            self.limo_x.publish(0.15)

       
    def callback(self, data):
            
        # Mise à jour de l'angle
        angle = self.data.limo_angle
        self.Twist.angular.z = angle

        # Mise à jour de x
        x = self.data.limo_x
        self.Twist.linear.x = x

        # Publish
        self.cmd_vel_pub.publish(self.Twist)

    def main(args):
        rospy.init_node('limo_etat')
        limo_etat = LimoEtat()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        
    if __name__ =='__main__':
        main(sys.argv)

