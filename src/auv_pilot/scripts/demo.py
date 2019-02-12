#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import OptionalPoint2D
from auv_common.msg import MoveGoal, MoveAction

# Initialization state
class InitializationState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok'])
        
    def execute(self, userdata):
        # We have to wait our action servers to initialize
        rate = rospy.Rate(10)
        rate.sleep()
        return 'ok'

def main():
    rospy.init_node('demo')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    forward = MoveGoal()
    forward.direction = MoveGoal.DIRECTION_FORWARD
    forward.value = 2000

    rotation = MoveGoal()
    rotation.direction = MoveGoal.ROTATE_YAW_CCW
    rotation.value = 500

    with sm:

        smach.StateMachine.add('Init', InitializationState(), transitions={'ok':'FORWARD_1'})

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
                                {'succeeded':'SUCCESS', 'preempted':'ABORTED', 'aborted':'ABORTED'})
    
    outcome = sm.execute()

if __name__ == '__main__':
    main()