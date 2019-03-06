#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
from auv_common.msg import MoveGoal, MoveAction


def create_qualification_fsm(qualification_duration):

    sm = smach.StateMachine(outcomes=['QUALIFICATION_OK', 'QUALIFICATION_FAILED'])

    with sm:

        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.value = qualification_duration * 1000 # Convert seconds to milliseconds
        forwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_4 # Maximal velocity
        forwardMoveGoal.holdIfInfinityValue = False

        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'ASCENT', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('ASCENT', common_states.create_diving_state(0), transitions={
            'succeeded':'QUALIFICATION_OK', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

    return sm