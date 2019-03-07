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

    