#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import OptionalPoint2D
from auv_common.msg import MoveByTimeActionGoal, MoveByTimeAction

def exploreGate(userData, gateMessage):
    return not gateMessage.hasPoint

def centerGate(userData, gateMessage):
    return not (gateMessage.hasPoint and abs(gateMessage.x) < 5)

def main():
    rospy.init_node('simple_simulation_fsm')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    with sm:

        smach.StateMachine.add('EXPLORE', 
                                smach_ros.MonitorState(
                                    '/gate',
                                    OptionalPoint2D,
                                    exploreGate),
                                {'valid':'EXPLORE', 'invalid':'SIDE_MOVE', 'preempted':'ABORTED'})

        sideMoveGoal = MoveByTimeActionGoal()
        sideMoveGoal.goal.direction = 2
        sideMoveGoal.goal.time = 0
        smach.StateMachine.add('SIDE_MOVE',
                                smach_ros.SimpleActionState(
                                    'auv_pilot',
                                    MoveByTimeAction,
                                    goal=sideMoveGoal),
                                {'succeeded':'GATE_MONITOR', 'preempted':'ABORTED', 'aborted':'ABORTED'})

        smach.StateMachine.add('GATE_MONITOR', 
                                smach_ros.MonitorState(
                                    '/gate',
                                    OptionalPoint2D,
                                    centerGate),
                                {'valid':'GATE_MONITOR', 'invalid':'FORWARD_MOVE', 'preempted':'ABORTED'})

        forwardMoveGoal = MoveByTimeActionGoal()
        forwardMoveGoal.goal.direction = 1
        forwardMoveGoal.goal.direction = 0
        smach.StateMachine.add('FORWARD_MOVE',
                                smach_ros.SimpleActionState(
                                    'auv_pilot',
                                    MoveByTimeAction,
                                    goal=forwardMoveGoal),
                                {'succeeded':'SUCCESS', 'preempted':'ABORTED', 'aborted':'ABORTED'})
                                
    
    outcome = sm.execute()

if __name__ == '__main__':
    main()