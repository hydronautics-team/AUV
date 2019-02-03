#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import OptionalPoint2D
from auv_common.msg import MoveGoal, MoveAction


# TODO: Try to implement through extending smach.StateMachine class
def create_gate_fsm():

    def exploreGate(self, userData, gateMessage):
        return not gateMessage.hasPoint


    def centerGate(self, userData, gateMessage):
        return not (gateMessage.hasPoint and abs(gateMessage.x) < 10)

    sm = smach.StateMachine(outcomes=['OK', 'FAILED'])

    with sm:

        sideMoveGoal = MoveGoal()
        sideMoveGoal.direction = MoveGoal.DIRECTION_LEFT
        sideMoveGoal.value = 0

        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.value = 10

        smach.StateMachine.add('SIDE_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=sideMoveGoal),
                                {'succeeded':'GATE_MONITOR', 'preempted':'FAILED', 'aborted':'FAILED'})

        smach.StateMachine.add('GATE_MONITOR', 
                                smach_ros.MonitorState(
                                    '/gate',
                                    OptionalPoint2D,
                                    centerGate),
                                {'valid':'GATE_MONITOR', 'invalid':'FORWARD_MOVE', 'preempted':'FAILED'})

        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'OK', 'preempted':'FAILED', 'aborted':'FAILED'})

    
    return sm