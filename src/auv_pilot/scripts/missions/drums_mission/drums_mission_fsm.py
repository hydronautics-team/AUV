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
from common import common_states


def create_drums_fsm():

    mode = rospy.get_param('~drum_mission_direction_mode', 'none').upper()

    rospy.loginfo(mode)
    if mode not in ['LEFT', 'RIGHT', 'MIDDLE']:
        rospy.loginfo('No executable mode specified. Allowed executable modes: left, right, middle.')
        mode = 'MIDDLE' #Default
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
            self.counter = 0

        def callback(self, matMessage):
            if matMessage.hasPoint and matMessage.x < 160:
                self.matDetected = True
            else:
                self.matDetected = False

        def execute(self, userdata):
            if self.matDetected:
                return 'MAT_DETECTED'
            else:
                self.counter += 1
                if self.counter > 7:
                    return 'FAILED'
                return 'NO_MAT_DETECTED'

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
        forwardLongMoveGoal.value = 18000
        forwardLongMoveGoal.holdIfInfinityValue = False


        smach.StateMachine.add('FORWARD_LONG_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardLongMoveGoal),
                               {'succeeded':'MAT_DETECTION', 'preempted':'DRUMS_FAILED', 'aborted':'DRUMS_FAILED'})


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

        smach.StateMachine.add('MAT_DETECTION', sm_sub,
                               transitions={'MAT_FRONT_CAM_NAVIGATION':'MAT_FRONT_CAM_NAVIGATION',
                                            'DRUMS_FAILED':'DRUMS_FAILED'})

        smach.StateMachine.add('MAT_FRONT_CAM_NAVIGATION', mat_front_cam_navigation.create_mat_front_cam_navigation_fsm(),
                              transitions={'HORIZONTAL_EDGE_DETECTED': 'MAT_BOTTOM_CAM_NAVIGATION', 'MAT_FRONT_CAM_NAVIGATION_FAILED': 'DRUMS_FAILED'})

        smach.StateMachine.add('MAT_BOTTOM_CAM_NAVIGATION', mat_bottom_cam_navigation.create_mat_bottom_cam_navigation_fsm(),
                              transitions={'BLUE_DRUM_DETECTED': 'DRUMS_NAVIGATION', 'RED_DRUM_DETECTED': 'DRUMS_NAVIGATION', 'MAT_BOTTOM_CAM_NAVIGATION_FAILED': 'DRUMS_FAILED'})

        smach.StateMachine.add('DRUMS_NAVIGATION', drums_navigation.create_drums_navigation_fsm(),
                               transitions={'DRUMS_NAVIGATION_OK': 'DRUMS_OK', 'DRUMS_NAVIGATION_FAILED': 'DRUMS_FAILED'})

    return sm


