#!/usr/bin/env python

import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal_shelf_1 = MoveBaseGoal()
    goal_shelf_1.target_pose.header.frame_id = "map"
    goal_shelf_1.target_pose.header.stamp = rospy.Time.now()
    goal_shelf_1.target_pose.pose.position.x = 1.51
    goal_shelf_1.target_pose.pose.position.y = 0.1
    goal_shelf_1.target_pose.pose.position.z = 0.01
    goal_shelf_1.target_pose.pose.orientation.z = 0.0
    goal_shelf_1.target_pose.pose.orientation.z = 0.0
    goal_shelf_1.target_pose.pose.orientation.w = -0.1

    client.send_goal(goal_shelf_1)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

    rospy.logerr("Waiting for the next goal")
    time.sleep(3)
    
    goal_shelf_2 = MoveBaseGoal()
    goal_shelf_2.target_pose.header.frame_id = "map"
    goal_shelf_2.target_pose.header.stamp = rospy.Time.now()
    goal_shelf_2.target_pose.pose.position.x = 2.05
    goal_shelf_2.target_pose.pose.position.y = 3.60
    goal_shelf_1.target_pose.pose.position.z = 0.01
    goal_shelf_2.target_pose.pose.orientation.z = 0.0
    goal_shelf_1.target_pose.pose.orientation.z = 0.0
    goal_shelf_2.target_pose.pose.orientation.w = -0.08

    client.send_goal(goal_shelf_2)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("goal_shelf_1 execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
