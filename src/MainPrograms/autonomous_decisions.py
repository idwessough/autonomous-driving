#! /usr/bin/env python2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
from darknet_ros_msgs.msg import BoundingBoxes
import message_filters 

def read_cameras():
    """_summary_: This function is used to read the images from the cameras (Darknet Depth) and to synchronize them.
    """    
    # Subscribe to ROSDARKNET to have the bounding boxes
    DarknetData = message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)
    # Subscribe to depth CAMERA to have the depth image
    DepthImage = message_filters.Subscriber("/camera/depth/image_raw", Image)
    
    print("STARTING DATA SYNCHRONIZATION BETWEEN CAMERA AND DARKNET")
    # Synchronize images 
    ts = message_filters.ApproximateTimeSynchronizer([DarknetData, DepthImage], queue_size=10, slop=2.5)
    # Using time sync to call the common callback function
    ts.registerCallback(images_callback)
    rospy.spin()

def images_callback(DarknetData, DepthImage):
    person_detected = False
    bridge = CvBridge() 
    print("{nb} Objects detected".format(len(DarknetData.bounding_boxes)))
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
        # Check if distance is close enough to react
        if nearest_distance <= reaction_distance:
            if object_class == "Stop": 
                action_todo = "STOP" 
            
            elif object_class == "CederPassage":
                action_todo = "SLOW" 

            # If there is a person near from the traffic sign
            elif object_class == "PassagePieton": 
                if person_distance-100 <= nearest_distance <= person_distance+100:
                    action_todo = "PASSAGEPIETON"
                else:
                    action_todo = "STOP" 
            # TODO : Add other cases after that
            elif box.Class == "TraficLight":
                if nearest_distance <= 200:
                    action_todo = "SLOW"
                print("soon charge")
        
            else:
                action_todo = "OBJECT PAS ENCORE PRIS EN CHARGE" 
                
        else:
            action_todo = "GO"
            print("continue")
        # publish action_todo in ros topic
        #limo_publisher = rospy.Publisher('/limo_action', String, queue_size=10)
        #limo_publisher.publish(action_todo)


            #action_todo = "STOP"
        # Process images...
        print(action_todo)

if __name__ == '__main__':
    rospy.init_node('my_node')
    try:
        read_cameras()
    except rospy.ROSInterruptException:
        pass
