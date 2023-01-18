#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


def handle_sign_pose(data, header_frame, sign_name,):
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header_frame
    t.child_frame_id = sign_name
    t.transform.translation.x = data.pose.position.x
    t.transform.translation.y = data.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = data.pose.orientation[0]
    t.transform.rotation.y = data.pose.orientation[1]
    t.transform.rotation.z = data.pose.orientation[2]
    t.transform.rotation.w = data.pose.orientation[3]

    br.sendTransform(t)
