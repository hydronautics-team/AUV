#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
import drums_navigation
import mat_front_cam_navigation
import mat_bottom_cam_navigation
from auv_common.msg import OptionalPoint2D
from auv_common.msg import MoveGoal, MoveAction
from auv_common.srv import TimerFsm
from common import common_states


def create_drums_fsm():

    mode = rospy.get_param('~drum_mission_direction_mode', 'none').upper()

    rospy.loginfo(mode)
    if mode not in ['LEFT', 'RIGHT', 'MIDDLE']:
        rospy.loginfo('No executable mode specified. Allowed executable modes: left, right, middle.')
        return
    rospy.loginfo("FSM drums_mission mode: " + mode)


    class lag_direction_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['RIGHT', 'LEFT', 'FAILED'])

        def execute(self, userdata):
            if mode == 'RIGHT':
                return 'RIGHT'
            elif mode == 'LEFT':
                return 'LEFT'
            else:
                return 'FAILED'

    class mat_detection_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['MAT_DETECTED', 'NO_MAT_DETECTED', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/mat/cam_front', OptionalPoint2D, self.callback)
            self.matDetected = False

        def callback(self, matMessage):
            if matMessage.hasPoint and matMessage.x < 160:
                self.matDetected = True

        def execute(self, userdata):
            if self.matDetected:
                return 'MAT_DETECTED'
            else:
                return 'NO_MAT_DETECTED'

    # gets called when ANY child state terminates
    def child_term_cb_detection(outcome_map):
        if outcome_map['MAT_DETECTION'] == 'MAT_FRONT_CAM_NAVIGATION':
            return True
        # terminate all running states if a lot of time passed
        if outcome_map['MAT_DETECTION_TIMER'] == 'TERMINATING':
            return True
        if outcome_map['MAT_DETECTION_TIMER'] == 'CALCULATING_TIME':
            return True
        if outcome_map['MAT_DETECTION'] == 'DRUMS_FAILED':
            return True
        return False

    # gets called when ALL child states are terminated
    def out_cb_detection(outcome_map):
        if outcome_map['MAT_DETECTION_TIMER'] == 'TERMINATING' and outcome_map['MAT_DETECTION'] != 'MAT_FRONT_CAM_NAVIGATION':
            return 'DRUMS_FAILED'
        if outcome_map['MAT_DETECTION_TIMER'] == 'CALCULATING_TIME' and outcome_map['MAT_DETECTION'] != 'MAT_FRONT_CAM_NAVIGATION':
            return 'Calculating_TIME_DETECTION'
        if outcome_map['MAT_DETECTION'] == 'MAT_FRONT_CAM_NAVIGATION':
            return 'SUCCESS'
        if outcome_map['MAT_DETECTION'] == 'DRUMS_FAILED':
            return 'DRUMS_FAILED'


    def child_term_cb_front_cam_navigation(outcome_map):
        if outcome_map['MAT_FRONT_CAM_NAVIGATION'] == 'HORIZONTAL_EDGE_DETECTED':
            return True
        if outcome_map['MAT_FRONT_CAM_NAVIGATION_TIMER'] == 'TERMINATING':
            return True
        if outcome_map['MAT_FRONT_CAM_NAVIGATION_TIMER'] == 'CALCULATING_TIME':
            return True
        if outcome_map['MAT_FRONT_CAM_NAVIGATION'] == 'MAT_FRONT_CAM_NAVIGATION_FAILED':
            return True
        return False

    def out_cb_front_cam_navigation(outcome_map):
        if outcome_map['MAT_FRONT_CAM_NAVIGATION_TIMER'] == 'TERMINATING' and outcome_map['MAT_FRONT_CAM_NAVIGATION'] != 'HORIZONTAL_EDGE_DETECTED':
            return 'DRUMS_FAILED'
        if outcome_map['MAT_FRONT_CAM_NAVIGATION_TIMER'] == 'CALCULATING_TIME' and outcome_map['MAT_FRONT_CAM_NAVIGATION'] != 'HORIZONTAL_EDGE_DETECTED':
            return 'Calculating_TIME_FRONT_CAM_NAVIGATION'
        if outcome_map['MAT_FRONT_CAM_NAVIGATION'] == 'HORIZONTAL_EDGE_DETECTED':
            return 'SUCCESS'
        if outcome_map['MAT_FRONT_CAM_NAVIGATION'] == 'MAT_FRONT_CAM_NAVIGATION_FAILED':
            return 'DRUMS_FAILED'


    def child_term_cb_bottom_cam_navigation(outcome_map):
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] == 'BLUE_DRUM_DETECTED':
            return True
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION_TIMER'] == 'TERMINATING':
            return True
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION_TIMER'] == 'CALCULATING_TIME':
            return True
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] == 'MAT_BOTTOM_CAM_NAVIGATION_FAILED':
            return True
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] == 'RED_DRUM_DETECTED':
            return True
        return False

    def out_cb_bottom_cam_navigation(outcome_map):
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION_TIMER'] == 'TERMINATING' and (outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] != 'BLUE_DRUM_DETECTED' or outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] != 'RED_DRUM_DETECTED'):
            return 'DRUMS_FAILED'
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION_TIMER'] == 'CALCULATING_TIME' and (outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] != 'BLUE_DRUM_DETECTED' or outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] != 'RED_DRUM_DETECTED'):
            return 'Calculating_TIME_BOTTOM_CAM_NAVIGATION'
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] == 'BLUE_DRUM_DETECTED':
            return 'SUCCESS'
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] == 'RED_DRUM_DETECTED':
            return 'SUCCESS'
        if outcome_map['MAT_BOTTOM_CAM_NAVIGATION'] == 'MAT_BOTTOM_CAM_NAVIGATION_FAILED':
            return 'DRUMS_FAILED'


    def child_term_cb_drum_cam(outcome_map):
        if outcome_map['DRUMS_NAVIGATION'] == 'DRUMS_NAVIGATION_OK':
            return True
        if outcome_map['DRUMS_NAVIGATION_TIMER'] == 'TERMINATING':
            return True
        if outcome_map['DRUMS_NAVIGATION_TIMER'] == 'CALCULATING_TIME':
            return True
        if outcome_map['DRUMS_NAVIGATION'] == 'DRUMS_NAVIGATION_FAILED':
            return True
        return False

    def out_cb_drum_cam(outcome_map):
        if outcome_map['DRUMS_NAVIGATION_TIMER'] == 'TERMINATING' and outcome_map['DRUMS_NAVIGATION'] != 'DRUMS_NAVIGATION_OK':
            return 'DRUMS_FAILED'
        if outcome_map['DRUMS_NAVIGATION_TIMER'] == 'CALCULATING_TIME' and outcome_map['DRUMS_NAVIGATION'] != 'DRUMS_NAVIGATION_OK':
            return 'Calculating_TIME_DRUMS_NAVIGATION'
        if outcome_map['DRUMS_NAVIGATION'] == 'DRUMS_NAVIGATION_OK':
            return 'SUCCESS'
        if outcome_map['DRUMS_NAVIGATION'] == 'DRUMS_NAVIGATION_FAILED':
            return 'DRUMS_FAILED'

    '''
    class mat_detection_timer_fsm(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['ERROR'])
            self.timer_flag = True
            #self.timer = rospy.ServiceProxy("timer_service", TimerFsm)
            #self.timerControl = self.timer(False)
            #print (self.timerControl.duration)

        def execute(self, userdata):
            rospy.sleep(30.0) # Test
            return 'ERROR'
    '''

    sm = smach.StateMachine(outcomes=['DRUMS_OK', 'DRUMS_FAILED'])

    with sm:

        leftMoveGoal = MoveGoal()
        leftMoveGoal.direction = MoveGoal.DIRECTION_LEFT
        leftMoveGoal.value = 1000
        leftMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        leftMoveGoal.holdIfInfinityValue = False

        rightMoveGoal = MoveGoal()
        rightMoveGoal.direction = MoveGoal.DIRECTION_RIGHT
        rightMoveGoal.value = 1000
        rightMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        rightMoveGoal.holdIfInfinityValue = False

        forwardLongMoveGoal = MoveGoal()
        forwardLongMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardLongMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardLongMoveGoal.value = 10000
        forwardLongMoveGoal.holdIfInfinityValue = False

        smach.StateMachine.add('FORWARD_LONG_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardLongMoveGoal),
                               {'succeeded':'DRUM_MISSION_DETECTION_FSM', 'preempted':'DRUMS_FAILED', 'aborted':'DRUMS_FAILED'})


        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['MAT_FRONT_CAM_NAVIGATION', 'DRUMS_FAILED'])

        with sm_sub:
            smach.StateMachine.add('MAT_DETECTION_CHECK', mat_detection_check(),
                                   transitions={'MAT_DETECTED':'MAT_FRONT_CAM_NAVIGATION',
                                                'NO_MAT_DETECTED':'LAG_DIRECTION_CONTROL',
                                                'FAILED':'DRUMS_FAILED'})

            smach.StateMachine.add('LAG_DIRECTION_CONTROL', lag_direction_control(),
                                   {'RIGHT':'RIGHT_MOVE', 'LEFT':'LEFT_MOVE', 'FAILED':'DRUMS_FAILED'})

            smach.StateMachine.add('LEFT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=leftMoveGoal),
                                   {'succeeded':'MAT_DETECTION_CHECK', 'preempted':'DRUMS_FAILED', 'aborted':'DRUMS_FAILED'})

            smach.StateMachine.add('RIGHT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=rightMoveGoal),
                                   {'succeeded':'MAT_DETECTION_CHECK', 'preempted':'DRUMS_FAILED', 'aborted':'DRUMS_FAILED'})
        '''-----------------------------------------------------------------------'''
        # Create the sub SMACH state machine
        sm_con_detection = smach.Concurrence(outcomes=['SUCCESS','DRUMS_FAILED', 'Calculating_TIME_DETECTION'],
                                             default_outcome='SUCCESS',
                                             child_termination_cb = child_term_cb_detection,
                                             outcome_cb = out_cb_detection)
        # Open the container
        with sm_con_detection:
            # Add states to the container
            smach.Concurrence.add('MAT_DETECTION', sm_sub)
            smach.Concurrence.add('MAT_DETECTION_TIMER', common_states.create_time_calculation_state(60, 1))


        smach.StateMachine.add('DRUM_MISSION_DETECTION_FSM', sm_con_detection,
                               transitions={'SUCCESS':'DRUM_MISSION_FRONT_CAM_NAVIGATION_FSM',
                                            'DRUMS_FAILED':'DRUMS_FAILED',
                                            'Calculating_TIME_DETECTION':'CIRCLE_DETECTION'})

        smach.StateMachine.add('CIRCLE_DETECTION', common_states.create_time_calculation_state(60, 1),
                               transitions={'CALCULATING_TIME':'DRUM_MISSION_DETECTION_FSM', 'TERMINATING':'DRUMS_FAILED'})
        '''-----------------------------------------------------------------------'''

        # Create the sub SMACH state machine
        sm_con_front_cam_navigation = smach.Concurrence(outcomes=['SUCCESS','DRUMS_FAILED', 'Calculating_TIME_FRONT_CAM_NAVIGATION'],
                                             default_outcome='SUCCESS',
                                             child_termination_cb = child_term_cb_front_cam_navigation,
                                             outcome_cb = out_cb_front_cam_navigation)
        # Open the container
        with sm_con_front_cam_navigation:
            # Add states to the container
            smach.Concurrence.add('MAT_FRONT_CAM_NAVIGATION', mat_front_cam_navigation.create_mat_front_cam_navigation_fsm())
            smach.Concurrence.add('MAT_FRONT_CAM_NAVIGATION_TIMER', common_states.create_time_calculation_state(60, 1))

        smach.StateMachine.add('DRUM_MISSION_FRONT_CAM_NAVIGATION_FSM', sm_con_front_cam_navigation,
                               transitions={'SUCCESS':'DRUM_MISSION_BOTTOM_CAM_NAVIGATION_FSM',
                                            'DRUMS_FAILED':'DRUMS_FAILED',
                                            'Calculating_TIME_FRONT_CAM_NAVIGATION':'CIRCLE_FRONT_CAM_NAVIGATION'})

        smach.StateMachine.add('CIRCLE_FRONT_CAM_NAVIGATION', common_states.create_time_calculation_state(60, 1),
                               transitions={'CALCULATING_TIME':'DRUM_MISSION_FRONT_CAM_NAVIGATION_FSM', 'TERMINATING':'DRUMS_FAILED'})
        '''-----------------------------------------------------------------------'''

        # Create the sub SMACH state machine
        sm_con_bottom_cam_navigation = smach.Concurrence(outcomes=['SUCCESS','DRUMS_FAILED', 'Calculating_TIME_BOTTOM_CAM_NAVIGATION'],
                                                        default_outcome='SUCCESS',
                                                        child_termination_cb = child_term_cb_bottom_cam_navigation,
                                                        outcome_cb = out_cb_bottom_cam_navigation)
        # Open the container
        with sm_con_bottom_cam_navigation:
            # Add states to the container
            smach.Concurrence.add('MAT_BOTTOM_CAM_NAVIGATION', mat_bottom_cam_navigation.create_mat_bottom_cam_navigation_fsm())
            smach.Concurrence.add('MAT_BOTTOM_CAM_NAVIGATION_TIMER', common_states.create_time_calculation_state(60, 1))

        smach.StateMachine.add('DRUM_MISSION_BOTTOM_CAM_NAVIGATION_FSM', sm_con_bottom_cam_navigation,
                               transitions={'SUCCESS':'DRUM_MISSION_DRUM_NAVIGATION_FSM',
                                            'DRUMS_FAILED':'DRUMS_FAILED',
                                            'Calculating_TIME_BOTTOM_CAM_NAVIGATION':'CIRCLE_BOTTOM_CAM_NAVIGATION'})

        smach.StateMachine.add('CIRCLE_BOTTOM_CAM_NAVIGATION', common_states.create_time_calculation_state(60, 1),
                               transitions={'CALCULATING_TIME':'DRUM_MISSION_BOTTOM_CAM_NAVIGATION_FSM', 'TERMINATING':'DRUMS_FAILED'})
        '''-----------------------------------------------------------------------'''

        # Create the sub SMACH state machine
        sm_con_drum_cam = smach.Concurrence(outcomes=['SUCCESS','DRUMS_FAILED', 'Calculating_TIME_DRUMS_NAVIGATION'],
                                                         default_outcome='SUCCESS',
                                                         child_termination_cb = child_term_cb_drum_cam,
                                                         outcome_cb = out_cb_drum_cam)
        # Open the container
        with sm_con_drum_cam:
            # Add states to the container
            smach.Concurrence.add('DRUMS_NAVIGATION', drums_navigation.create_drums_navigation_fsm())
            smach.Concurrence.add('DRUMS_NAVIGATION_TIMER', common_states.create_time_calculation_state(60, 1))

        smach.StateMachine.add('DRUM_MISSION_DRUM_NAVIGATION_FSM', sm_con_drum_cam,
                               transitions={'SUCCESS':'DRUMS_OK',
                                            'DRUMS_FAILED':'DRUMS_FAILED',
                                            'Calculating_TIME_DRUMS_NAVIGATION':'CIRCLE_DRUM_NAVIGATION'})

        smach.StateMachine.add('CIRCLE_DRUM_NAVIGATION', common_states.create_time_calculation_state(60, 1),
                               transitions={'CALCULATING_TIME':'DRUM_MISSION_DRUM_NAVIGATION_FSM', 'TERMINATING':'DRUMS_FAILED'})

        '''
        smach.Concurrence.add('MAT_FRONT_CAM_NAVIGATION', mat_front_cam_navigation.create_mat_front_cam_navigation_fsm(),
                              transitions={'HORIZONTAL_EDGE_DETECTED': 'MAT_BOTTOM_CAM_NAVIGATION', 'MAT_FRONT_CAM_NAVIGATION_FAILED': 'DRUMS_FAILED'})
        
        smach.Concurrence.add('MAT_BOTTOM_CAM_NAVIGATION', mat_bottom_cam_navigation.create_mat_bottom_cam_navigation_fsm(),
                              transitions={'BLUE_DRUM_DETECTED': 'DRUMS_NAVIGATION', 'RED_DRUM_DETECTED': 'DRUMS_NAVIGATION', 'MAT_BOTTOM_CAM_NAVIGATION_FAILED': 'DRUMS_FAILED'})
        
        smach.StateMachine.add('DRUMS_NAVIGATION', drums_navigation.create_drums_navigation_fsm(),
                               transitions={'DRUMS_NAVIGATION_OK': 'DRUMS_OK', 'DRUMS_NAVIGATION_FAILED': 'DRUMS_FAILED'})
        '''
    return sm


