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

    gate_count = 0
    def exploreGate(userData, gateMessage):
        global gate_count
        if gateMessage.isPresent:
            gate_count += 1
            if gate_count > 5 and gateMessage.xCenter < 10:
                gate_count = 0
                return False
            else:
                gate_count = 0
                return True
        else:
            gate_count = 0
            return True

    sm = smach.StateMachine(outcomes=['GATE_OK', 'GATE_FAILED'])

    with sm:

        firstForwardMoveGoal = MoveGoal()
        firstForwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        firstForwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_3
        firstForwardMoveGoal.value = 9500

        lagDirection = rospy.get_param('~lagDirection', 'LEFT')
        sideMoveGoal = MoveGoal()
        if lagDirection == 'LEFT':
            sideMoveGoal.direction = MoveGoal.DIRECTION_LEFT
        else:
            sideMoveGoal.direction = MoveGoal.DIRECTION_RIGHT
        sideMoveGoal.value = 0
        sideMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        sideMoveGoal.holdIfInfinityValue = False

        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardMoveGoal.value = 4000

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
                                    exploreGate),
                                {'valid':'GATE_MONITOR', 'invalid':'FORWARD_MOVE', 'preempted':'GATE_FAILED'})

        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'GATE_OK', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

    
    return sm

    