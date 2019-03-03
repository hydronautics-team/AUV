#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.srv import *
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


class IMUResetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OK'])

    def execute(self, userdata):
        reset_service = rospy.ServiceProxy("reset_service", ResetCmd)
        response = reset_service()
        rospy.sleep(0.5)
        return 'OK'


# Creates diving state (depth in centimeters)
def create_diving_state(depth):
    dive = DiveGoal()
    dive.depth = depth
    return smach_ros.SimpleActionState('dive', DiveAction, goal=dive)
