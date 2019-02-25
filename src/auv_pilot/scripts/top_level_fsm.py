#! /usr/bin/env python

import rospy
import smach
import smach_ros
from qualification import qualification_fsm
from missions import missions_fsm
from demo import demo_fsm


def main():
    rospy.init_node('top_level_fsm')

    mode = rospy.get_param('~mode', 'none').upper()
    rospy.loginfo(mode)
    if mode not in ['QUALIFICATION', 'MISSIONS', 'DEMO']:
        rospy.loginfo('No executable mode specified. Allowed executable modes: qualification, missions, demo.')
        return
    rospy.loginfo("FSM mode: " + mode)

    if mode != 'DEMO':
        if not (rospy.has_param('~launch_delay') or rospy.has_param('~dive_delay') or rospy.has_param('~initial_depth')):
            rospy.logerr("launch_delay, dive_delay, initial_depth must be set!")
            raise
        launch_delay = int(rospy.get_param('~launch_delay'))
        dive_delay = int(rospy.get_param('~dive_delay'))
        initial_depth = int(rospy.get_param('~initial_depth'))

    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])

    with sm:
        
        if mode == 'QUALIFICATION':
            smach.StateMachine.add('QUALIFICATION', qualification_fsm.create_qualification_fsm(launch_delay, dive_delay, initial_depth), 
                transitions={'QUALIFICATION_OK': 'SUCCEEDED', 'QUALIFICATION_FAILED': 'FAILED'})

        elif mode == 'MISSIONS':
            smach.StateMachine.add('MISSIONS', missions_fsm.create_missions_fsm(launch_delay, dive_delay, initial_depth), 
                transitions={'MISSIONS_OK': 'SUCCEEDED', 'MISSIONS_FAILED': 'FAILED'})

        elif mode == 'DEMO':
            smach.StateMachine.add('DEMO', demo_fsm.create_demo_fsm(), transitions={'DEMO_OK': 'SUCCEEDED', 'DEMO_FAILED': 'FAILED'})
        
    server = smach_ros.IntrospectionServer('top_level_fsm', sm, '/fsm/top_level_fsm')
    server.start()
    
    outcome = sm.execute()

    rospy.spin()
    server.stop()


if __name__ == '__main__':
    main()