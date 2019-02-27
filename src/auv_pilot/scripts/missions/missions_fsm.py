#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
import gate_mission
from drums_mission import drums_mission_fsm

def create_missions_fsm(launch_delay, dive_delay, initial_depth):

    sm = smach.StateMachine(outcomes=['MISSIONS_OK', 'MISSIONS_FAILED'])

    with sm:
        smach.StateMachine.add('LAUNCH_DELAY', common_states.WaitState(launch_delay), transitions={'OK': 'DIVE_DELAY'})
        smach.StateMachine.add('DIVE_DELAY', common_states.WaitState(dive_delay), transitions={'OK': 'DIVE'})
        smach.StateMachine.add('DIVE', common_states.create_diving_state(initial_depth), transitions={
            'succeeded':'GATE_MISSION', 'preempted':'MISSIONS_FAILED', 'aborted':'MISSIONS_FAILED'})
        smach.StateMachine.add('GATE_MISSION', gate_mission.create_gate_fsm(), transitions={'GATE_OK': 'DRUMS_MISSION', 'GATE_FAILED': 'MISSIONS_FAILED'})
        smach.StateMachine.add('DRUMS_MISSION', drums_mission_fsm.create_drums_fsm(), transitions={'DRUMS_OK': 'MISSIONS_OK', 'DRUMS_FAILED': 'MISSIONS_FAILED'})
        #??smach.StateMachine.add('DIVE', common_states.create_diving_state(initial_depth), transitions={
            #'succeeded':'DRUMS_MISSION', 'preempted':'MISSIONS_FAILED', 'aborted':'MISSIONS_FAILED'})
        #smach.StateMachine.add('BALL_DROP', common_states.create_diving_state(initial_depth), transitions={
            #'succeeded':'DRUMS_MISSION', 'preempted':'MISSIONS_FAILED', 'aborted':'MISSIONS_FAILED'})
        #smach.StateMachine.add('RISE', common_states.create_diving_state(initial_depth), transitions={
            #'succeeded':'DRUMS_MISSION', 'preempted':'MISSIONS_FAILED', 'aborted':'MISSIONS_FAILED'})

    return sm

