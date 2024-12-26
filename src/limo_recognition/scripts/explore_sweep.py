#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,PointStamped
from sensor_msgs.msg import LaserScan
import random
from math import pi
import math

res_flag = False
OD_flag = False

def rotate_2pi(publisher):
    # rotate 2pi
    move = Twist()
    move.angular.z = 2*pi
    time_now = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec()<time_now+3:
        publisher.publish(move)
    rospy.loginfo('Rotating...................')
    return

class move_to_target:
    def __init__(self):
        self.vel_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
    def main_move(self, angle, depth, size):
        global res_flag, OD_flag
        near_enough_threshold = 0.03
        if size > 20000:
            print('rescue success')
            res_flag = True
            OD_flag = False
            rospy.sleep(10)
            # rotate_2pi(self.vel_cmd)
            return
        move = Twist()
        Kp = 1
        move.linear.x = 0.1
        move.angular.z = -Kp*angle
        print("angular.z", angle, move.angular.z)
        self.vel_cmd.publish(move)
        
        
        
class random_explore:
    def __init__(self):
        global OD_flag
        rospy.init_node('random_explore', anonymous=True)
        OD_flag = False
        self.very_near = False
        self.data = PointStamped()
        self.data_time = 0
        self.vel_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.check_obstacle)
        rospy.Subscriber('/object_detect_pose', PointStamped, self.object_detected_callback)
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
            self.very_near = True
            
        
        for r in front_ranges:
            # print(r)
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
        move_forward.linear.x = 0.2
        move_forward.angular.z = 0
        
        turn_around = Twist()
        # turn a random angle between -90 and 0 degrees
        turn_around.angular.z = angle 
        return move_forward, turn_around
    
    def main_loop(self):
        global OD_flag
        # main loop of the robot
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.very_near == True:
                self.go_back()
                self.very_near = False
            # OD_flag = False
            
            ######################################################random explore#########################################
            if OD_flag == False:
                angle = random.uniform(-pi/4, pi/4)
                #set the angle to -pi/2 or pi/2 if there is an obstacle in front of the robot
                angle = -pi/8 if angle < 0 else pi/8
                if self.if_obstacle:
                    # if there is an obstacle in front of the robot, turn around
                    while self.if_obstacle:
                        self.vel_cmd.publish(self.random_move(angle)[1])
                    # rospy.sleep(0.1)
                else:
                    # otherwise, move forward
                    self.vel_cmd.publish(self.random_move(angle)[0])
                
                rate.sleep()
            else:
                print('Object_there!!!!!!!!')
                self.data_time += 1
                x = self.data.point.x
                size = self.data.point.y
                depth = self.data.point.z
                if depth != 0:
                    angle = math.atan(x/depth)
                else:
                    angle = 0
                print('angle and depth are:',angle, depth, size)
                move_to_target().main_move(angle, depth, size)

                if self.data_time > 10:
                    OD_flag = False
            #########################################################random explore#######################################
     
    def object_detected_callback(self,data):
        global OD_flag
        rospy.loginfo("Moving to target.....")
        OD_flag = True
        self.data = data
        self.data_time = 0        
        
            
                

        
                       
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel_cmd.publish(Twist())
        rospy.sleep(1)
    def go_back(self):
        print("Obstacle Very Near!!!!")
        out_cmd = Twist()
        out_cmd.linear.x = -0.8
        self.vel_cmd.publish(out_cmd)
        rospy.sleep(0.5)
        # rotate_2pi(self.vel_cmd)
        
    
if __name__ == '__main__':
    
    re = random_explore()
    # re.go_back()
    re.main_loop()
    rospy.spin()
