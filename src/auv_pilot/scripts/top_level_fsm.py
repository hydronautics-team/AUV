#! /usr/bin/env python

import rospy
import smach
import smach_ros
import std_srvs.srv
from common import common_states
from qualification import qualification_fsm
from missions import missions_fsm
from demo import demo_fsm


def main():
    rospy.init_node('top_level_fsm')

    mode = rospy.get_param('~mode', 'none').upper()
    rospy.loginfo(mode)
    if mode not in ['QUALIFICATION_SIMPLE', 'QUALIFICATION_VISION', 'MISSIONS', 'DEMO']:
        rospy.loginfo('No executable mode specified. Allowed executable modes: qualification_simple, qualification_vision, missions, demo.')
        return
    rospy.loginfo("FSM mode: " + mode)

    if mode != 'DEMO':
        if not (rospy.has_param('~diveDelay') or rospy.has_param('~initialDepth') or
                rospy.has_param('~imuReset')):
            rospy.logerr("dive_delay, initial_depth, imu_reset must be set!")
            raise
        dive_delay = int(rospy.get_param('~diveDelay'))
        initial_depth = int(rospy.get_param('~initialDepth'))
        imu_reset = bool(rospy.get_param('~imuReset'))
        rospy.loginfo("Dive delay: " + str(dive_delay) + " Initial depth: " + str(initial_depth) + " IMU reset: " + str(imu_reset))
    if mode == 'QUALIFICATION_SIMPLE':
        if not (rospy.has_param('~qualificationDuration')):
            rospy.logerr("qualification_duration parameter must be set for qualification simple mode")
            raise
        qualification_duration = int(rospy.get_param('~qualificationDuration'))
        rospy.loginfo("Qualification forward move duration: " + str(qualification_duration))

    notifyOnFinish = rospy.get_param('~notifyOnFinish', False)

    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])

    with sm:

        if mode != 'DEMO':
            if imu_reset:
                smach.StateMachine.add('IMU_INIT', common_states.IMUInitState(), transitions={'OK': 'DIVE_DELAY'})
            smach.StateMachine.add('DIVE_DELAY', common_states.WaitState(dive_delay), transitions={'OK': 'STABILIZATION_INIT'})
            smach.StateMachine.add('STABILIZATION_INIT', common_states.StabilizationInitState(), transitions={'OK': 'DIVE'})
            smach.StateMachine.add('DIVE', common_states.create_diving_state(initial_depth),
                                   transitions={'succeeded':mode, 'preempted':'SUCCEEDED', 'aborted':'FAILED'})

        if mode == 'QUALIFICATION_SIMPLE':
            smach.StateMachine.add('QUALIFICATION_SIMPLE', qualification_fsm.create_qualification_simple_fsm(qualification_duration),
                                   transitions={'QUALIFICATION_OK': 'SUCCEEDED', 'QUALIFICATION_FAILED': 'FAILED'})

        elif mode == 'QUALIFICATION_VISION':
            smach.StateMachine.add('QUALIFICATION_VISION', qualification_fsm.create_qualification_vision_fsm(),
                                   transitions={'QUALIFICATION_OK': 'SUCCEEDED', 'QUALIFICATION_FAILED': 'FAILED'})

        elif mode == 'MISSIONS':
            smach.StateMachine.add('MISSIONS', missions_fsm.create_missions_fsm(),
                                   transitions={'MISSIONS_OK': 'SUCCEEDED', 'MISSIONS_FAILED': 'FAILED'})

        elif mode == 'DEMO':
            smach.StateMachine.add('DEMO', demo_fsm.create_demo_fsm(), transitions={'DEMO_OK': 'SUCCEEDED', 'DEMO_FAILED': 'FAILED'})

    server = smach_ros.IntrospectionServer('top_level_fsm', sm, '/fsm/top_level_fsm')
    server.start()

    outcome = sm.execute()
    rospy.loginfo('FSM finished')

    if notifyOnFinish:
        fsm_finished_service = rospy.ServiceProxy('global_fsm_finished', std_srvs.srv.Trigger)
        fsm_finished_service()

    rospy.spin()
    server.stop()


if __name__ == '__main__':
    main()