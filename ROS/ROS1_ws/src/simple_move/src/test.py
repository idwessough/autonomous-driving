#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = -1.54
    goal.pose.position.y = -1.33
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    while not rospy.is_shutdown():
        rospy.loginfo("publishing goal to move_base_simple/goal")
        goal_publisher.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
