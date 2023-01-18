#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


def handle_baselink_pose(data, baselink_name):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = baselink_name
    t.transform.translation.x = data.pose.pose.position.x
    t.transform.translation.y = data.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = data.pose.pose.orientation.x
    t.transform.rotation.y = data.pose.pose.orientation.y
    t.transform.rotation.z = data.pose.pose.orientation.z
    t.transform.rotation.w = data.pose.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_to_baselink')
    baselink_name = "base_link"
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_baselink_pose, baselink_name)
    rospy.spin()