#! /usr/bin/env python

import rospy
import smach
import smach_ros
import auv_common.srv
from common import common_states
from qualification import qualification_fsm
from missions import missions_fsm
from demo import demo_fsm

launched = False
receivedMode = None

def launch_callback(req):
    global launched
    global receivedMode
    mode = req.mode.upper()
    if mode not in ['QUALIFICATION_SIMPLE', 'QUALIFICATION_VISION', 'MISSIONS']:
        return auv_common.srv.LaunchCmdResponse(False,
                                                'No executable mode specified. Allowed executable modes: qualification_simple, qualification_vision, missions.')
    if launched:
        return auv_common.srv.LaunchCmdResponse(False, 'Already launched')
    receivedMode = mode
    launched = True
    return auv_common.srv.LaunchCmdResponse(True, 'OK')


def main():
    global receivedMode
    global launched

    rospy.init_node('top_level_fsm')

    startCondition = rospy.get_param('~startCondition', 'none').upper()
    if startCondition not in ['AUTO', 'TRIGGER']:
        rospy.logerr('Unknown start condition. Available start conditions: auto, trigger')

    if startCondition == 'TRIGGER':
        rospy.loginfo('Waiting for launch trigger')
        fsm_start_service = rospy.Service('fsm_start', auv_common.srv.LaunchCmd, launch_callback)
    else:
        launched = True

    while not rospy.is_shutdown():

        if launched:

            if startCondition == 'AUTO':
                mode = rospy.get_param('~mode', 'none').upper()
            else:
                mode = receivedMode

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

            sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])

            with sm:

                if mode != 'DEMO':
                    if imu_reset:
                        if startCondition == 'TRIGGER':
                             smach.StateMachine.add('SIGNAL', common_states.create_signal_state(),
                                           transitions={'succeeded':'IMU_INIT', 'preempted':'SUCCEEDED', 'aborted':'FAILED'})
                        smach.StateMachine.add('IMU_INIT', common_states.IMUInitState(), transitions={'OK': 'DIVE_DELAY'})
                    else:
                        if startCondition == 'TRIGGER':
                            smach.StateMachine.add('SIGNAL', common_states.create_signal_state(),
                                           transitions={'succeeded':'DIVE_DELAY', 'preempted':'SUCCEEDED', 'aborted':'FAILED'})
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

            server.stop()

            return


if __name__ == '__main__':
    main()