#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import LaserScan
import random
from math import pi
import math


def rotate_2pi(publisher):
    # rotate 2pi
    move = Twist()
    move.angular.z = 2*pi/3
    publisher.publish(move)
    rospy.sleep(3)
    rospy.loginfo('Rotating...................')
    return

class move_to_target:
    def __init__(self):
        self.vel_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
    def main_move(self, angle, depth):
        near_enough_threshold = 0.03
        if depth < near_enough_threshold:
            print('rescue success')
            rotate_2pi(self.vel_cmd)
            return
        move = Twist()
        Kp = 1
        move.linear.x = 0.3
        move.angular.z = Kp*angle
        self.vel_cmd.publish(move)
        
        
        
class random_explore:
    def __init__(self):
        rospy.init_node('random_explore', anonymous=True)
        self.OD_flag = False
        self.vel_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.check_obstacle)
        rospy.sleep(1)
        rospy.on_shutdown(self.shutdown)
        self.if_obstacle = False
    
    def zero_vote(self,list):
        # list: []
        cnt = 0
        for _ in list:
            if _ == 0:
                cnt+=1
        
        if cnt/len(list) > 0.9:
            return True
        else:
            return False
        
    def check_obstacle(self, data):
        # check if there is any obstacle in front of the robot
        
        # get the range of the laser scan
        ranges = data.ranges

        #get the area of the laser scan in front of the robot
        numbers = len(ranges)
        # print(numbers)
        front_ranges = ranges[(numbers//2-40):(numbers//2+40)]
        # print("Front ranges:", front_ranges[len(front_ranges)//2])
        # check if there is any obstacle in front of the robot
        # check if front_ranges are all zeros
        if self.zero_vote(front_ranges):
            print("Obstacle Very Near!!!!")
            out_cmd = Twist()
            out_cmd.linear.x = -1
            self.vel_cmd.publish(out_cmd)
            rospy.sleep(0.15)
            rotate_2pi(self.vel_cmd)
            
        
        for r in front_ranges:
            print(r)
            if r < 0.5 and r!=0.0:
                print("Obstacle detected!")
                self.if_obstacle = True
                rospy.sleep(0.01)
                return
            
        self.if_obstacle = False
        return
        
        
        
    
    def random_move(self,angle=0):
        # generate a random move
        
        move_forward = Twist()
        move_forward.linear.x = 0.5
        # move_forward.angular.z = random.uniform(-2,2)
        
        turn_around = Twist()
        # turn a random angle between -90 and 0 degrees
        turn_around.angular.z = angle 
        return move_forward, turn_around
    
    def main_loop(self):
        # main loop of the robot
        
        while not rospy.is_shutdown():
            self.OD_flag = False
            rospy.Subscriber('/object_detect_pose', PoseStamped, self.object_detected_callback)
            ######################################################random explore#########################################
            if self.OD_flag == False:
                angle = random.uniform(-pi/2, pi/2)
                #set the angle to -pi/2 or pi/2 if there is an obstacle in front of the robot
                angle = -pi/4 if angle < 0 else pi/4
                if self.if_obstacle:
                    # if there is an obstacle in front of the robot, turn around
                    while self.if_obstacle:
                        self.vel_cmd.publish(self.random_move(angle)[1])
                    # rospy.sleep(0.1)
                else:
                    # otherwise, move forward
                    self.vel_cmd.publish(self.random_move(angle)[0])
                
                rospy.sleep(0.01)
            else:
                continue
            #########################################################random explore#######################################
     
    def object_detected_callback(self,data):
        rospy.loginfo("Moving to target.....")
        self.OD_flag = True
        data = PoseStamped()
        x = data.pose.position.x
        depth = data.pose.position.z
        angle = math.atan(x/depth)
        print('angle and depth are:',angle,depth)
        move_to_target().main_move(angle, depth)
            
                

        
                       
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel_cmd.publish(Twist())
        rospy.sleep(1)
    
if __name__ == '__main__':
    
    re = random_explore()
    re.main_loop()
