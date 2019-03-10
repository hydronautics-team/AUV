#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import Gate
from auv_common.msg import MoveGoal, MoveAction, CenteringAction, CenteringGoal
from common import common_states


LAG_DIRECTION = None
lagDirectionParam = rospy.get_param('~lagDirection', 'LEFT')
if lagDirectionParam == 'LEFT':
    LAG_DIRECTION = MoveGoal.DIRECTION_LEFT
else:
    LAG_DIRECTION = MoveGoal.DIRECTION_RIGHT
rospy.loginfo('Lag direction: '+  str(LAG_DIRECTION))

FIRST_MARCH_TIME = int(rospy.get_param('~firstMarchTime', '5500'))
SECOND_MARCH_TIME = int(rospy.get_param('~secondMarchTime', '12000'))


def create_gate_fsm_by_timings():
    sm = smach.StateMachine(outcomes=['GATE_OK', 'GATE_FAILED'])
    with sm:

        if FIRST_MARCH_TIME != 0:
            smach.StateMachine.add('MARCH_1', common_states.create_move_state(
                MoveGoal.DIRECTION_FORWARD, FIRST_MARCH_TIME, MoveGoal.VELOCITY_LEVEL_2),
                                   {'succeeded':'LAG_INF', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('LAG_1', common_states.create_move_state(
            LAG_DIRECTION, 4500, MoveGoal.VELOCITY_LEVEL_2),
                               {'succeeded':'MARCH_2', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('MARCH_2', common_states.create_move_state(
            MoveGoal.DIRECTION_FORWARD, SECOND_MARCH_TIME, MoveGoal.VELOCITY_LEVEL_2),
                               {'succeeded':'GATE_OK', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})
    return sm


def create_gate_fsm_centering_simple():

    gate_count = 0

    def explore_gate(userData, gateMessage):
        global gate_count
        if gateMessage.isPresent:
            gate_count += 1
            rospy.loginfo("Number: " + str(gate_count))
            if gate_count > 4:
                return False
            else:
                return True
        else:
            gate_count = 0
            return True

    sm = smach.StateMachine(outcomes=['GATE_OK', 'GATE_FAILED'])
    with sm:

        if FIRST_MARCH_TIME != 0:
            smach.StateMachine.add('MARCH_1', common_states.create_move_state(
                MoveGoal.DIRECTION_FORWARD, FIRST_MARCH_TIME, MoveGoal.VELOCITY_LEVEL_3),
                                   {'succeeded':'LAG_INF', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('LAG_INF', common_states.create_move_state(
            LAG_DIRECTION, MoveGoal.VALUE_INFINITY, MoveGoal.VELOCITY_LEVEL_1, False),
                               {'succeeded':'GATE_MONITOR', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('GATE_MONITOR',
                               smach_ros.MonitorState(
                                   '/gate',
                                   Gate,
                                   explore_gate),
                               {'valid':'GATE_MONITOR', 'invalid':'MARCH_2', 'preempted':'GATE_FAILED'})

        smach.StateMachine.add('MARCH_2', common_states.create_move_state(
            MoveGoal.DIRECTION_FORWARD, SECOND_MARCH_TIME, MoveGoal.VELOCITY_LEVEL_3),
                               {'succeeded':'GATE_OK', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

    return sm


def create_gate_fsm_centering_vision():

    gate_count = 0

    def explore_gate(userData, gateMessage):
        global gate_count
        if gateMessage.isPresent:
            gate_count += 1
            if gate_count > 5:
                return False
            else:
                return True
        else:
            gate_count = 0
            return True

    centeringGoal = CenteringGoal()
    centeringGoal.targetSource = '/gate'
    if LAG_DIRECTION == MoveGoal.DIRECTION_LEFT:
        centeringGoal.initialDirection = CenteringGoal.LEFT
    else:
        centeringGoal.initialDirection = CenteringGoal.RIGHT

    sm = smach.StateMachine(outcomes=['GATE_OK', 'GATE_FAILED'])
    with sm:

        if FIRST_MARCH_TIME != 0:
            smach.StateMachine.add('MARCH_1', common_states.create_move_state(
                MoveGoal.DIRECTION_FORWARD, FIRST_MARCH_TIME, MoveGoal.VELOCITY_LEVEL_2),
                                   {'succeeded':'LAG_INF', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('LAG_INF', common_states.create_move_state(
            LAG_DIRECTION, MoveGoal.VALUE_INFINITY, MoveGoal.VELOCITY_LEVEL_2, False),
                               {'succeeded':'GATE_MONITOR', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('GATE_MONITOR',
                               smach_ros.MonitorState(
                                   '/gate',
                                   Gate,
                                   explore_gate),
                               {'valid':'GATE_MONITOR', 'invalid':'CENTERING', 'preempted':'GATE_FAILED'})

        smach.StateMachine.add('CENTERING',
                                smach_ros.SimpleActionState(
                                    'centering',
                                    CenteringAction,
                                    goal=centeringGoal),
                                {'succeeded':'MARCH_2', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

        smach.StateMachine.add('MARCH_2', common_states.create_move_state(
            MoveGoal.DIRECTION_FORWARD, SECOND_MARCH_TIME, MoveGoal.VELOCITY_LEVEL_2),
                               {'succeeded':'GATE_OK', 'preempted':'GATE_FAILED', 'aborted':'GATE_FAILED'})

    return sm

    