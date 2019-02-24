#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.msg import DiveGoal, DiveAction

# TODO: Create more common states

# Creates delay (in seconds)
class WaitState(smach.State):
    def __init__(self, delay):
        smach.State.__init__(self, outcomes=['OK'])
        self.delay = delay
        
    def execute(self, userdata):
        if self.delay == 0:
            return 'OK'
        rate = rospy.Rate(1.0 / self.delay)
        rate.sleep()
        return 'OK'


# Creates diving state (depth in centimeters)
def create_diving_state(depth):
    dive = DiveGoal()
    dive.depth = depth
    return smach_ros.SimpleActionState('dive', DiveAction, goal=dive)