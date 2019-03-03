#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import Gate
from auv_common.msg import MoveGoal, MoveAction, CenteringAction, CenteringGoal


# TODO: Try to implement through extending smach.StateMachine class
def create_gate_fsm():

    def exploreGate(userData, gateMessage):
        return not gateMessage.hasPoint


    def centerGate(userData, gateMessage):
        return not (gateMessage.isPresent and abs(gateMessage.xCenter) < 10)

    sm = smach.StateMachine(outcomes=['GATE_OK', 'GATE_FAILED'])

    with sm:

        firstForwardMoveGoal = MoveGoal()
        firstForwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        firstForwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        firstForwardMoveGoal.value = 5000

        sideMoveGoal = MoveGoal()
        sideMoveGoal.direction = MoveGoal.DIRECTION_LEFT
        sideMoveGoal.value = 0
        sideMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_2
        sideMoveGoal.holdIfInfinityValue = False

        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardMoveGoal.value = 10000

        smach.StateMachine.add('FIRST_FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=firstForwardMoveGoal),
                               {'succeeded':'SIDE_MOVE', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('SIDE_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=sideMoveGoal),
                                {'succeeded':'GATE_MONITOR', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('GATE_MONITOR', 
                                smach_ros.MonitorState(
                                    '/gate',
                                    Gate,
                                    centerGate),
                                {'valid':'GATE_MONITOR', 'invalid':'FORWARD_MOVE', 'preempted':'GATE_FAILED'})

        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'GATE_OK', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

    
    return sm


def create_new_gate_fsm():

    gate_count = 0
    def exploreGate(userData, gateMessage):
        global gate_count
        if gateMessage.isPresent:
            gate_count += 1
            if gate_count > 2:
                gate_count = 0
                return False
        else:
            gate_count = 0
            return True

    sm = smach.StateMachine(outcomes=['GATE_OK', 'GATE_FAILED'])

    with sm:

        firstForwardMoveGoal = MoveGoal()
        firstForwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        firstForwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        firstForwardMoveGoal.value = 0
        firstForwardMoveGoal.holdIfInfinityValue = False

        centeringGoal = CenteringGoal()
        centeringGoal.targetSource = '/gate'
        centeringGoal.initialDirection = CenteringGoal.NONE

        stopGoal = MoveGoal()
        stopGoal.direction = MoveGoal.STOP
        stopGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        stopGoal.value = 0
        stopGoal.holdIfInfinityValue = False

        secondForwardMoveGoal = MoveGoal()
        secondForwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        secondForwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_4
        secondForwardMoveGoal.value = 10000

        smach.StateMachine.add('FIRST_FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=firstForwardMoveGoal),
                               {'succeeded':'GATE_MONITOR', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('GATE_MONITOR',
                               smach_ros.MonitorState(
                                   '/gate',
                                   Gate,
                                   exploreGate),
                               {'valid':'STOP', 'invalid':'STOP', 'preempted':'GATE_FAILED'})

        smach.StateMachine.add('STOP',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=stopGoal),
                               {'succeeded':'CENTERING', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('CENTERING',
                                smach_ros.SimpleActionState(
                                    'centering',
                                    CenteringAction,
                                    goal=centeringGoal),
                                {'succeeded':'SECOND_FORWARD_MOVE', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('SECOND_FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=secondForwardMoveGoal),
                               {'succeeded':'GATE_OK', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})


    return sm
    