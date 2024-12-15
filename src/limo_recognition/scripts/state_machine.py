#!/usr/bin/python3 

# a state machine useing smach
# control a robot to first explore the maze and build maps
# memorize the map and target locations
# then go to the target locations one by one
# global path planning is used to find the path to the target locations
# local path planning using mpc and visual servoing

import rospy
import smach
import smach.state
import smach_ros
import actionlib
import random
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2


# define the state using smach_ros
# the state is a class with a method execute
# the method execute is called when the state is active
# the method execute should return 'succeeded', 'aborted' or 'preempted'
# the method execute should have a parameter userdata
# the method execute should return the


class Exploring(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.exploring_time = 120 #seconds
    def execute(self, userdata):
        rospy.loginfo("Exploring the maze")
        return 'succeeded'

class GlobalPathPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Global path planning")
        return 'succeeded'
    
class PathExecution(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Path execution")
        return 'succeeded'

class VisualServoing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Visual servoing")
        return 'succeeded'

    

    
class GoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    
    def execute(self, userdata):
        rospy.loginfo("Going home")
        return 'succeeded'

class ExploringMaze():
    def __init__(self) -> None:
        pass
    
    
    
    def state_machine(self):
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('EXPLORING', Exploring(),
                                   transitions={'succeeded':'GLOBAL_PATH_PLANNING',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            #add state "global path planning"
            smach.StateMachine.add('GLOBAL_PATH_PLANNING', GlobalPathPlanning(),
                                   transitions={'succeeded':'PATH_EXECUTION',
                                                'aborted':'GO_HOME',
                                                'preempted':'preempted'})
            smach.StateMachine.add('PATH_EXECUTION', PathExecution(),
                                   transitions={'succeeded':'VISUAL_SERVOING',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            
            smach.StateMachine.add('VISUAL_SERVOING', VisualServoing(),
                                      transitions={'succeeded':'GLOBAL_PATH_PLANNING',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
            # 
            
            # add a MonitorState to check if the objects are all rescued
            
            smach.StateMachine.add('GO_HOME', GoHome(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
        # Execute the state machine
        outcome = sm.execute()
        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()
        
        
if __name__ == '__main__':
    rospy.init_node('exploring_maze')
    em = ExploringMaze()
    em.state_machine()