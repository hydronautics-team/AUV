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
        rate = rospy.Rate(1)
        rate.sleep()
        return 'ok'

def main():
    rospy.init_node('demo')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    forward = MoveGoal()
    forward.direction = MoveGoal.DIRECTION_FORWARD
    forward.value = 10

    rotation = MoveGoal()
    rotation.direction = MoveGoal.ROTATE_YAW_CCW
    rotation.value = 3

    with sm:

        smach.StateMachine.add('Init', InitializationState(), transitions={'ok':'FIRST_FORWARD'})

        smach.StateMachine.add('FIRST_FORWARD',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'FIRST_ROTATION', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('FIRST_ROTATION',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'SECOND_FORWARD', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('SECOND_FORWARD',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'SECOND_ROTATION', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('SECOND_ROTATION',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'THIRD_FORWARD', 'preempted':'ABORTED', 'aborted':'ABORTED'})


        smach.StateMachine.add('THIRD_FORWARD',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=forward),
                                {'succeeded':'THIRD_ROTATION', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('THIRD_ROTATION',
                                smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rotation),
                                {'succeeded':'FIRST_FORWARD', 'preempted':'ABORTED', 'aborted':'ABORTED'})
                                
    
    outcome = sm.execute()

if __name__ == '__main__':
    main()