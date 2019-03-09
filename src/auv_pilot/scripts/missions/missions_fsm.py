#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
import gate_mission
import drums_mission


def create_missions_fsm():

    if rospy.has_param('/gateFsmMode'):
        gate_fsm_mode = rospy.get_param('/gateFsmMode')
    else:
        if not rospy.has_param('~gateFsmMode'):
            rospy.logerr('Gate FSM mode not specified, available modes: timings, simple, vision')
            raise
    gate_fsm_mode = rospy.get_param('~gateFsmMode')

    rospy.loginfo('Gate FSM mode: ' + gate_fsm_mode)

    sm = smach.StateMachine(outcomes=['MISSIONS_OK', 'MISSIONS_FAILED'])

    with sm:

        if gate_fsm_mode == 'timings':
            smach.StateMachine.add('GATE_MISSION', gate_mission.create_gate_fsm_by_timings(), transitions={'GATE_OK': 'MISSIONS_OK', 'GATE_FAILED': 'MISSIONS_FAILED'})
        elif gate_fsm_mode == 'simple':
            smach.StateMachine.add('GATE_MISSION', gate_mission.create_gate_fsm_centering_simple(), transitions={'GATE_OK': 'MISSIONS_OK', 'GATE_FAILED': 'MISSIONS_FAILED'})
        elif gate_fsm_mode == 'vision':
            smach.StateMachine.add('GATE_MISSION', gate_mission.create_gate_fsm_centering_vision(), transitions={'GATE_OK': 'MISSIONS_OK', 'GATE_FAILED': 'MISSIONS_FAILED'})

        # TODO THIS IS ONLY FOR DEBUGGING
        #smach.StateMachine.add('DRUMS_MISSION', drums_mission.create_drums_fsm(), transitions={'DRUMS_OK': 'MISSIONS_OK', 'DRUMS_FAILED': 'MISSIONS_FAILED'})

    return sm

