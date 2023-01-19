#! /usr/bin/env python2

import rospy
import sys
# tf imports
import tf2_ros
import tf2_geometry_msgs
import tf
import sign_tf2_broadcaster
import tf_conversions

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from darknet_ros_msgs.msg import BoundingBoxes
import message_filters 
import numpy as np

camera_angle = 67.9

class sign_identification:
    def __init__(self):
        print("Initializing sign_identification node")
        self.flag = 0
        self.traffic_sign = None
        # config rate 
        self.rate = rospy.Rate(rospy.get_param("/rate/sign_identification")) 
        # init subscriber/publisher
        # Subscribe to ROSDARKNET to have the bounding boxes
        self.DarknetData = message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)
        # Subscribe to depth CAMERA to have the depth image
        self.DepthImage = message_filters.Subscriber("/camera/depth/image_raw", Image)
        #publisher for actions to follow for limo
        self.limo_publisher = rospy.Publisher("/limo_action", String, queue_size=10)
        # Tf buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Synchronize images 
        ts = message_filters.ApproximateTimeSynchronizer([self.DarknetData, self.DepthImage], queue_size=10, slop=2.5)
        # Using time sync to call the common callback function
        ts.registerCallback(self.images_callback)

    def images_callback(self, DarknetData, DepthImage):
        person_detected = False
        bridge = CvBridge() 
        print("{nb} Objects detected".format(nb=len(DarknetData.bounding_boxes)))
        depth_image = bridge.imgmsg_to_cv2(DepthImage)
        print(depth_image.shape)
        nearest_distance = 100000
        nearest_object = None
        # Loop through bounding boxes
        for box in DarknetData.bounding_boxes: #box.xmin, box.xmax, box.ymin, box.ymax 
            
            #rospy.loginfo(box) 
            # Compute center of bounding box
            Xbounding_center = (box.xmin + box.xmax)/2
            Ybounding_center = (box.ymin + box.ymax)/2
            #object_distance = depth_image[box.ymin:box.ymax, box.xmin:box.xmax].mean()
            object_distance = depth_image[Ybounding_center, Xbounding_center]
            # Avoid 0 values (no depth made by an hadware offset)
            if object_distance != 0: 
                # Check if person is detected
                if box.Class == "Person":
                    person_detected = True
                    person_distance = object_distance
                # Check if object is closer than the previous one
                elif nearest_distance > object_distance:
                    nearest_distance = object_distance
                    nearest_object = box
                else:
                    continue
        # Special case if a person is detected near from the car
        if person_detected:
            if person_distance <= 420:
                action_todo = "PERSON"
        # If there is another nearest object
        if nearest_object is not None: 
            print("Nearest object : {nearest_object.Class} at {nearest_distance} mm".format(nearest_object=nearest_object, nearest_distance=nearest_distance))
            reaction_distance = 750    
            object_class = nearest_object.Class
            # Check if the object is the same as the previous one
            if object_class == self.traffic_sign:
                self.flag = 1
            else:
                self.flag = 0
                self.traffic_sign = object_class

            # Check if distance is close enough to react
            if nearest_distance <= reaction_distance:
                if object_class == "Stop": 
                    action_todo = "STOP" 
                
                elif object_class == "CederPassage":
                    action_todo = "SLOW" 

                # If there is a person near from the traffic sign
                elif object_class == "PassagePieton": 
                    if person_detected and person_distance-100 <= nearest_distance <= person_distance+100:
                        action_todo = "PASSAGEPIETON"
                    else:
                        pass 
                
                elif object_class == "rouge": 
                        action_todo = "REDLIGHT"
                
                elif object_class == "vert":
                    action_todo = "GREENLIGHT"
                    
                elif object_class == "VirageDroite":
                    action_todo = "RIGHT"
            
                elif object_class == "RoutePrioritaire":
                    action_todo = "PRIORITY"
                
                # TODO : Add other cases after that    
                else:
                    print("soon change")
                    action_todo = "OBJECT PAS ENCORE PRIS EN CHARGE" 
                
                # compute coordinates of point
                X, Y = self.compute_distance(nearest_distance)

                # Create new sign tf in odom frame
                if self.flag == 0:
                    self.Create_tf(X, Y, action_todo)
                    self.flag = 1
                

            else:
                action_todo = "GO"
                print("continue")

            rospy.loginfo("Publishing action : {action_todo}".format(action_todo=action_todo))
            # publish action_todo in ros topic
            self.limo_publisher.publish(action_todo)
            rospy.loginfo("Published action : {action_todo}".format(action_todo=action_todo))


    def Create_tf(self, X, Y, action_todo):
        # poseStamped in camera_depth_frame
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_depth_frame"
        msg.pose.position.x = X
        msg.pose.position.y = Y
        msg.pose.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        # transpose msg from /camera_depth to /odom frame
        transform = self.tfBuffer.lookup_transform('odom', msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        #pb quaternion vs euler a check
        pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)

        # Create new sign tf in respect to odom
        sign_tf2_broadcaster.handle_sign_pose(pose_transformed, "odom", action_todo)

    def compute_distance(self, nearest_distance):
        # Compute X and Y coordinates in frame camera_depth for the center of the bounding box thanks to trigonometry
        object_angle = (camera_angle/2)# - (camera_angle * (nearest_object.xmin + nearest_object.xmax)/(2*camera_width))
        X = abs(nearest_distance * np.sin(object_angle))
        Y = abs(nearest_distance * np.cos(object_angle))
        print("X = {X} mm, Y = {Y} mm".format(X=X, Y=Y))
        return X, Y

    def run(self):
            while not rospy.is_shutdown():
                self.rate.sleep()

def main(args):
    rospy.init_node('sign_identification')
    Sign_id = sign_identification()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ =='__main__':
    main(sys.argv)
