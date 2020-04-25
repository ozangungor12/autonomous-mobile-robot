#!/usr/bin/env python
import rospy
from math import sqrt, atan2, degrees, radians
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveRobot():
    def __init__(self):
        rospy.init_node("move_to_goal")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.r = rospy.Rate(10)
        self.position = Point()
        self.position.x = 0
        self.position.y = 0
        self.yaw = 0
        self.goal = Point()
        self.goal.x = 3
        self.goal.y = 4

        self.run()

    def odom_callback(self, msg):    
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        
    def get_goal_distance_and_angle(self, goal_x, goal_y):
        distance = sqrt(pow(goal_x - self.position.x,2) + pow(goal_y - self.position.y,2))
        theta = atan2(goal_y-self.position.y, goal_x-self.position.x)
        theta_degrees = degrees(theta)
        return distance, theta_degrees

    def rotate_to_goal(self, goal_x, goal_y):
        angle_threshold = 0.02
        angle_goal = atan2(goal_y-self.position.y, goal_x-self.position.x)
        angle_diff = (angle_goal - self.yaw)
        
        kp = 0.9
        move_cmd = Twist()

        while abs(angle_diff) > angle_threshold:
            move_cmd.angular.z = kp * angle_diff
            self.vel_pub.publish(move_cmd)
            angle_goal = atan2(goal_y-self.position.y, goal_x-self.position.x)
            angle_diff = (angle_goal - self.yaw)
            self.r.sleep()

    def move_linear(self, goal_x, goal_y):
        
        distance = sqrt(pow(goal_x - self.position.x,2) + pow(goal_y - self.position.y,2))
        distance_threshold = distance * 0.04

        kp = 0.9
        move_cmd = Twist()
        
        while (distance > distance_threshold): 
            move_cmd.linear.x = kp * distance
            self.vel_pub.publish(move_cmd)
            distance = sqrt(pow(goal_x - self.position.x,2) + pow(goal_y - self.position.y,2))
            self.r.sleep()

    def mix_robot_move(self, goal_x, goal_y):
        angle_threshold = 0.02
        angle_goal = atan2(goal_y-self.position.y, goal_x-self.position.x)
        angle_diff = (angle_goal - self.yaw)

        distance = sqrt(pow(goal_x - self.position.x,2) + pow(goal_y - self.position.y,2))
        distance_threshold = distance * 0.04

        kp_a = 0.9
        kp_l = 0.5
        move_cmd = Twist()

        while (abs(angle_diff) > angle_threshold and distance > distance_threshold):
            move_cmd.angular.z = kp_a * angle_diff
            move_cmd.linear.x = kp_l * distance

            self.vel_pub.publish(move_cmd)

            angle_goal = atan2(goal_y-self.position.y, goal_x-self.position.x)
            angle_diff = (angle_goal - self.yaw)
            distance = sqrt(pow(goal_x - self.position.x,2) + pow(goal_y - self.position.y,2))
            self.r.sleep()

    def run(self):
        for i in range(1):
            goal_x = -1
            goal_y= -1
            
            distance, theta_degrees = self.get_goal_distance_and_angle(goal_x,goal_y)
            print(distance, theta_degrees)
            
            if(theta_degrees % 90 == 0):
                self.rotate_to_goal(goal_x, goal_y)
                self.move_linear(goal_x, goal_y)

            else:
                if (90 < theta_degrees < 270):
                    self.rotate_to_goal(0,1)
                elif (-90 > theta_degrees > -270):
                    self.rotate_to_goal(0, -1)
                self.mix_robot_move(goal_x, goal_y)
            

if __name__ == "__main__":
    try:
        MoveRobot()
    except rospy.ROSInterruptException:
        pass



