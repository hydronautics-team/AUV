#! /usr/bin/env python

import rospy
import smach
import smach_ros
from init import init_states
from missions import gate_mission


def main():
    rospy.init_node('main_fsm')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'ABORTED'])

    with sm:

        smach.StateMachine.add('INIT', init_states.SimpleInitState(10), transitions={'OK': 'GATE_MISSION'})
        smach.StateMachine.add('GATE_MISSION', gate_mission.create_gate_fsm(), transitions={'OK': 'SUCCESS', 'FAILED': 'ABORTED'})
        
    server = smach_ros.IntrospectionServer('main_fsm', sm, '/fsm/main_fsm')
    server.start()
    
    outcome = sm.execute()

    rospy.spin()
    server.stop()



if __name__ == '__main__':
    main()