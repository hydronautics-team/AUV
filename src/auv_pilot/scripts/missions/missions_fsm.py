#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
import gate_mission
import drums_mission


def create_missions_fsm():

    sm = smach.StateMachine(outcomes=['MISSIONS_OK', 'MISSIONS_FAILED'])

    with sm:
        smach.StateMachine.add('GATE_MISSION', gate_mission.create_gate_fsm(), transitions={'GATE_OK': 'MISSIONS_OK', 'GATE_FAILED': 'MISSIONS_FAILED'})
        # TODO THIS IS ONLY FOR DEBUGGING
        #smach.StateMachine.add('DRUMS_MISSION', drums_mission.create_drums_fsm(), transitions={'DRUMS_OK': 'MISSIONS_OK', 'DRUMS_FAILED': 'MISSIONS_FAILED'})

    return sm

