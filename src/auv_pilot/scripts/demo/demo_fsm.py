#! /usr/bin/env python

import rospy
import smach
import smach_ros
from common import common_states
from auv_common.msg import MoveGoal, MoveAction


def create_demo_fsm():

    sm = smach.StateMachine(outcomes=['DEMO_OK', 'DEMO_FAILED'])

    forward = MoveGoal()
    forward.direction = MoveGoal.DIRECTION_FORWARD
    forward.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
    forward.value = 2000

    rotation = MoveGoal()
    rotation.direction = MoveGoal.ROTATE_YAW_CCW
    rotation.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
    rotation.value = 800

    with sm:
        
        smach.StateMachine.add('DELAY', common_states.WaitState(10), transitions={'OK': 'FORWARD_1'})

        smach.StateMachine.add('FORWARD_1',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'ROTATION_1', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})

        smach.StateMachine.add('ROTATION_1',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FORWARD_2', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})

        smach.StateMachine.add('FORWARD_2',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'ROTATION_2', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})

        smach.StateMachine.add('ROTATION_2',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FORWARD_3', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})


        smach.StateMachine.add('FORWARD_3',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'ROTATION_3', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})

        smach.StateMachine.add('ROTATION_3',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FORWARD_4', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})

        smach.StateMachine.add('FORWARD_4',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'FORWARD_1', 'preempted':'DEMO_FAILED', 'aborted':'DEMO_FAILED'})

    return sm
