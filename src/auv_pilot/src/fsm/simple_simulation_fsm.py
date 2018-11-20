#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.msg import OptionalPoint2D



def main():
    rospy.init_node('simple_simulation_fsm')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    with sm:
        smach.StateMachine.add('EXPLORE', 
                                smach_ros.MonitorState(
                                    '/gate',
                                    OptionalPoint2D,
                                    {'valid':'EXPLORE', 'invalid':'SUCCESS'}
                                ))
    
    outcome = sm.execute()

if __name__ == '__main__':
    main()