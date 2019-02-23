#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from init import init_states
from auv_common.msg import OptionalPoint2D
from auv_common.msg import MoveGoal, MoveAction


def main():
    rospy.init_node('demo')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    forward = MoveGoal()
    forward.direction = MoveGoal.DIRECTION_FORWARD
    forward.velocityLevel = MoveGoal.VELOCITY_LEVEL_3
    forward.value = 2000

    rotation = MoveGoal()
    rotation.direction = MoveGoal.ROTATE_YAW_CCW
    rotation.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
    rotation.value = 800

    with sm:

        smach.StateMachine.add('INIT', init_states.SimpleInitState(10), transitions={'OK': 'DIVE'})
        # Depth in millimeters
        smach.StateMachine.add('DIVE', init_states.create_diving_state(1000), transitions={'succeeded':'FORWARD_1', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('FORWARD_1',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'ROTATION_1', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('ROTATION_1',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FORWARD_2', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('FORWARD_2',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'ROTATION_2', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('ROTATION_2',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FORWARD_3', 'preempted':'ABORTED', 'aborted':'ABORTED'})


        smach.StateMachine.add('FORWARD_3',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'ROTATION_3', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('ROTATION_3',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FORWARD_4', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('FORWARD_4',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'FORWARD_1', 'preempted':'ABORTED', 'aborted':'ABORTED'})
    
    outcome = sm.execute()

if __name__ == '__main__':
    main()