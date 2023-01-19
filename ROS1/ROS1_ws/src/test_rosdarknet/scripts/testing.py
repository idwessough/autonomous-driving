#!/usr/bin/env python
import pandas as pd
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.point_cloud2 import PointCloud2 as pc2
# Beta script emulating autonomous behaviour without real data inputc
 
def callback(data): 
    if type(data) == BoundingBoxes:
        return data
        
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s from ROSDarknet", data)



def depth_callback(data):
    return data



def process_boxes(bounding_boxes, depth_image):
    for box in data.bounding_boxes:
        rospy.loginfo(f"Object detected: {box.Class} with {box.probability} probability" ) 
        #box.xmin, box.xmax, box.ymin, box.ymax
        print(box.xmin, box.xmax, box.ymin, box.ymax)
        rospy.loginfo(box) 
        if box.Class == "Stop":
            bounding_center = (box.xmin + box.xmax)/2
        return action_todo

 

def main():
    while not rospy.is_shutdown():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
        process_boxes(data, depth_image)
        rospy.spin()

if __name__ == '__main__':
    testing_mode = True
    if testing_mode:
        try:
            main()
            except rospy.ROSInterruptException:
                pass
        else:
            print("el") 