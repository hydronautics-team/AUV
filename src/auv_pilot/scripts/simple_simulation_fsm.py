#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import OptionalPoint2D
from auv_common.msg import MoveByTimeGoal, MoveByTimeAction

def exploreGate(userData, gateMessage):
    return not gateMessage.hasPoint

def centerGate(userData, gateMessage):
    return not (gateMessage.hasPoint and abs(gateMessage.x) < 5)

def main():
    rospy.init_node('simple_simulation_fsm')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    with sm:

        sideMoveGoal = MoveByTimeGoal()
        sideMoveGoal.direction = 4
        sideMoveGoal.time = 0
        smach.StateMachine.add('SIDE_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time_server',
                                    MoveByTimeAction,
                                    goal=sideMoveGoal),
                                {'succeeded':'GATE_MONITOR', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('GATE_MONITOR', 
                                smach_ros.MonitorState(
                                    '/gate',
                                    OptionalPoint2D,
                                    centerGate),
                                {'valid':'GATE_MONITOR', 'invalid':'FORWARD_MOVE', 'preempted':'ABORTED'})

        forwardMoveGoal = MoveByTimeGoal()
        forwardMoveGoal.direction = 1
        forwardMoveGoal.time = 5
        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'move_by_time_server',
                                    MoveByTimeAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'SUCCESS', 'preempted':'ABORTED', 'aborted':'ABORTED'})
                                
    
    outcome = sm.execute()

if __name__ == '__main__':
    main()