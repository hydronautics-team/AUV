#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.msg import DiveGoal, DiveAction


class SimpleInitState(smach.State):
    def __init__(self, delay):
        smach.State.__init__(self, outcomes=['OK'])
        self.delay = delay
        
    def execute(self, userdata):
        # We have to wait our action servers to initialize
        rate = rospy.Rate(1.0 / self.delay)
        rate.sleep()
        return 'OK'


def create_diving_state(depth):
    dive = DiveGoal()
    dive.depth = depth
    return smach_ros.SimpleActionState('dive', DiveAction, goal=dive)

