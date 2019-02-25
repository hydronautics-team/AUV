#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states


def create_qualification_fsm(launch_delay, dive_delay, initial_depth):

    sm = smach.StateMachine(outcomes=['QUALIFICATION_OK', 'QUALIFICATION_FAILED'])

    with sm:
        smach.StateMachine.add('LAUNCH_DELAY', common_states.WaitState(launch_delay), transitions={'OK': 'DIVE_DELAY'})
        smach.StateMachine.add('DIVE_DELAY', common_states.WaitState(dive_delay), transitions={'OK': 'DIVE'})
        smach.StateMachine.add('DIVE', common_states.create_diving_state(initial_depth), transitions={
            'succeeded':'QUALIFICATION_OK', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})
        # TODO: Add forward move
    return sm