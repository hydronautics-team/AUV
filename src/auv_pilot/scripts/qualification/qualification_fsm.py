#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
from auv_common.msg import Gate
from auv_common.msg import MoveGoal, MoveAction, CenteringAction, CenteringGoal


def create_qualification_simple_fsm(qualification_duration):

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

        smach.StateMachine.add('ASCENT', common_states.create_diving_state(-20), transitions={
            'succeeded':'QUALIFICATION_OK', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

    return sm


def create_qualification_vision_fsm():

    gate_count = 0
    def exploreGate(userData, gateMessage):
        global gate_count
        if gateMessage.isPresent:
            gate_count += 1
            if gate_count > 10:
                gate_count = 0
                return False
        else:
            gate_count = 0
            return True

    sm = smach.StateMachine(outcomes=['QUALIFICATION_OK', 'QUALIFICATION_FAILED'])

    with sm:

        firstForwardMoveGoal = MoveGoal()
        firstForwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        firstForwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_4
        firstForwardMoveGoal.value = 0
        firstForwardMoveGoal.holdIfInfinityValue = False

        centeringGoal = CenteringGoal()
        centeringGoal.targetSource = '/gate'
        centeringGoal.initialDirection = CenteringGoal.NONE

        secondForwardMoveGoal = MoveGoal()
        secondForwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        secondForwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_4
        secondForwardMoveGoal.value = 10000

        smach.StateMachine.add('FIRST_FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=firstForwardMoveGoal),
                               {'succeeded':'GATE_MONITOR', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('GATE_MONITOR',
                               smach_ros.MonitorState(
                                   '/gate',
                                   Gate,
                                   exploreGate),
                               {'valid':'GATE_MONITOR', 'invalid':'CENTERING', 'preempted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('CENTERING',
                               smach_ros.SimpleActionState(
                                   'centering',
                                   CenteringAction,
                                   goal=centeringGoal),
                               {'succeeded':'SECOND_FORWARD_MOVE', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('SECOND_FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=secondForwardMoveGoal),
                               {'succeeded':'ASCENT', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})

        smach.StateMachine.add('ASCENT', common_states.create_diving_state(-20), transitions={
            'succeeded':'QUALIFICATION_OK', 'preempted':'QUALIFICATION_FAILED', 'aborted':'QUALIFICATION_FAILED'})


    return sm