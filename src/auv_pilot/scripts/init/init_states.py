#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.msg import MoveGoal, MoveAction


class SimpleInitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OK'])
        
    def execute(self, userdata):
        # We have to wait our action servers to initialize
        rate = rospy.Rate(1)
        rate.sleep()
        return 'OK'


def create_diving_state(depth):
    diveMove = MoveGoal()
    diveMove.direction = MoveGoal.DIRECTION_DOWN
    diveMove.value = (depth * 1000.0) / 3000.0
    diveMove.velocityLevel = MoveGoal.VELOCITY_LEVEL_2
    diveMove.holdIfInfinityValue = False

    return smach_ros.SimpleActionState('move_by_time', MoveAction, goal=diveMove)

