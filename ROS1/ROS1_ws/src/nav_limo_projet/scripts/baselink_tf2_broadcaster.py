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
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, data.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

"""header: 
  seq: 91652
  stamp: 
    secs: 1673973345
    nsecs: 624881672
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: -0.000443911303076
      y: 0.00135954430578
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.00261799088742
      w: 0.999996573056
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: -0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
type
nav_msgs/Odometry
"""

if __name__ == '__main__':
    rospy.init_node('tf2_baselink_broadcaster')
    baselink_name = "base_link"
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_baselink_pose)
    rospy.spin()