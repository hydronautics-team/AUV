#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
from auv_common.msg import MoveGoal, MoveAction


def create_qualification_fsm(launch_delay, dive_delay, initial_depth):

    sm = smach.StateMachine(outcomes=['QUALIFICATION_OK', 'QUALIFICATION_FAILED'])

    with sm:

        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.value = 30000 # 10,000 msec = 10 sec
        forwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1 # Maximal velocity
        forwardMoveGoal.holdIfInfinityValue = False


        smach.StateMachine.add('LAUNCH_DELAY', common_states.WaitState(launch_delay), transitions={'OK': 'DIVE_DELAY'})

        smach.StateMachine.add('DIVE_DELAY', common_states.WaitState(dive_delay), transitions={'OK': 'DIVE'})

        smach.StateMachine.add('DIVE', common_states.create_diving_state(initial_depth), transitions={
            'succeeded':'FORWARD_MOVE', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'ASCENT', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('ASCENT', common_states.create_diving_state(0), transitions={
            'succeeded':'QUALIFICATION_OK', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

    return sm