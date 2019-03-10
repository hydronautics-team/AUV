#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import DrumsCoordinates
from auv_common.msg import MoveGoal, MoveAction
#from std_msgs.msg import Float64, Bool

#timer_pub = rospy.Publisher('/drum_navigation_FSM/time', Float64)
#timer_sub = rospy.Subscriber('/drum_navigation_FSM/timer_status', JointState, transform_callback)

def create_drums_navigation_fsm():

    '''
    def color_check(userData, drumMessage):
        return not (drumMessage.hasRedDrum or drumMessage.hasBlueDrum)
    '''

    def drum_check(userData, drumMessage):
        return not (drumMessage.hasRedDrum or drumMessage.hasBlueDrum)

    class sleep_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['SUCCESS'])

        def execute(self, userdata):
            rospy.sleep(10.) # sleep for 10 seconds
            return 'SUCCESS'

    class lag_direction_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['X_PositionIsOK', 'NeedRightMove', 'NeedLeftMove', 'FAILED', 'NO_DRUM_DETECTED'])

            self.subscriber = rospy.Subscriber('/drums/drum', DrumsCoordinates, self.callback)
            self.lagMoveNeeded = False
            self.rightMove = False
            self.leftMove = False
            self.hasRedDrum = False
            self.hasBlueDrum = False

        def callback(self, drumMessage):
            if drumMessage.hasBlueDrum:
                self.hasBlueDrum = True
                self.hasRedDrum = False
                if abs(drumMessage.BlueDrum1x) > 35:
                    self.lagMoveNeeded = True
                    if drumMessage.BlueDrum1x > 0:
                        self.rightMove = True
                        self.leftMove = False
                    else:
                        self.leftMove = True
                        self.rightMove = False
                else:
                    self.lagMoveNeeded = False

            elif drumMessage.hasRedDrum:
                self.hasRedDrum = True
                self.hasBlueDrum = False
                if abs(drumMessage.RedDrum1x) > 35:
                    self.lagMoveNeeded = True
                    if drumMessage.RedDrum1x > 0:
                        self.rightMove = True
                        self.leftMove = False
                    else:
                        self.leftMove = True
                        self.rightMove = False
                else:
                    self.lagMoveNeeded = False
            else:
                self.lagMoveNeeded = False
                self.hasRedDrum = False
                self.hasBlueDrum = False

        def execute(self, userdata):
            if self.lagMoveNeeded:
                if self.rightMove:
                    return 'NeedRightMove'
                if self.leftMove:
                    return 'NeedLeftMove'
            elif self.hasRedDrum or self.hasBlueDrum:
                return 'X_PositionIsOK'
            else:
                return 'NO_DRUM_DETECTED'

    class march_direction_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['Y_PositionIsOK', 'NeedForwardMove', 'NeedBackwardMove', 'FAILED', 'NO_DRUM_DETECTED'])

            self.subscriber = rospy.Subscriber('/drums/drum', DrumsCoordinates, self.callback)
            self.marchMoveNeeded = False
            self.forwardMove = False
            self.backwardMove = False
            self.hasRedDrum = False
            self.hasBlueDrum = False

        def callback(self, drumMessage):
            if drumMessage.hasBlueDrum:
                self.hasBlueDrum = True
                self.hasRedDrum = False
                if abs(drumMessage.BlueDrum1y) > 35:
                    self.marchMoveNeeded = True
                    if drumMessage.BlueDrum1y > 0:
                        self.forwardMove = True
                        self.backwardMove = False
                    else:
                        self.backwardMove = True
                        self.forwardMove = False
                else:
                    self.marchMoveNeeded = False

            elif drumMessage.hasRedDrum:
                self.hasRedDrum = True
                self.hasBlueDrum = False
                if abs(drumMessage.RedDrum1y) > 35:
                    self.marchMoveNeeded = True
                    if drumMessage.RedDrum1y > 0:
                        self.forwardMove = True
                        self.backwardMove = False
                    else:
                        self.backwardMove = True
                        self.forwardMove = False
                else:
                    self.marchMoveNeeded = False
            else:
                self.marchMoveNeeded = False
                self.hasRedDrum = False
                self.hasBlueDrum = False


        def execute(self, userdata):
            if self.marchMoveNeeded:
                if self.forwardMove:
                    return 'NeedForwardMove'
                if self.backwardMove:
                    return 'NeedBackwardMove'
            elif self.hasRedDrum or self.hasBlueDrum:
                return 'Y_PositionIsOK'
            else:
                return 'NO_DRUM_DETECTED'

    sm = smach.StateMachine(outcomes=['DRUMS_NAVIGATION_OK', 'DRUMS_NAVIGATION_FAILED'])

    with sm:

        leftMoveGoal = MoveGoal()
        leftMoveGoal.direction = MoveGoal.DIRECTION_LEFT
        leftMoveGoal.value = 500
        leftMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        leftMoveGoal.holdIfInfinityValue = False

        rightMoveGoal = MoveGoal()
        rightMoveGoal.direction = MoveGoal.DIRECTION_RIGHT
        rightMoveGoal.value = 500
        rightMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        rightMoveGoal.holdIfInfinityValue = False


        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardMoveGoal.value = 500
        forwardMoveGoal.holdIfInfinityValue = False

        backwardsMoveGoal = MoveGoal()
        backwardsMoveGoal.direction = MoveGoal.DIRECTION_BACKWARDS
        backwardsMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        backwardsMoveGoal.value = 500
        backwardsMoveGoal.holdIfInfinityValue = False

        '''
        # DEBUG!!!
        smach.StateMachine.add('COLOR_CHECK',
                               smach_ros.MonitorState(
                                   '/drums/drum',
                                   DrumsCoordinates,
                                   color_check),
                               {'invalid':'DRUM_CENTERING', 'valid':'COLOR_CHECK', 'preempted':'DRUMS_NAVIGATION_FAILED'})
        '''

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['CENTERED', 'FAILED'])

        with sm_sub:
            smach.StateMachine.add('DRUM_NAVIGATION_LAG', lag_direction_control(),
                                   transitions={'X_PositionIsOK':'DRUM_NAVIGATION_MARCH',
                                                'NeedLeftMove':'LEFT_MOVE',
                                                'NeedRightMove':'RIGHT_MOVE',
                                                'FAILED':'FAILED',
                                                'NO_DRUM_DETECTED':'WAITING_DRUM_DETECTION_MSG_LAG'})

            # MonitorState outcome switches from valid to invalid
            smach.StateMachine.add('WAITING_DRUM_DETECTION_MSG_LAG',
                                   smach_ros.MonitorState(
                                       '/drums/drum',
                                       DrumsCoordinates,
                                       drum_check),
                                   {'invalid':'DRUM_NAVIGATION_LAG', 'valid':'WAITING_DRUM_DETECTION_MSG_LAG', 'preempted':'FAILED'})

            #smach.StateMachine.add('SLEEP_LAG', sleep_control(),
                                   #transitions={'SUCCESS':'DRUM_NAVIGATION_LAG'})

            smach.StateMachine.add('LEFT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=leftMoveGoal),
                                   {'succeeded':'DRUM_NAVIGATION_LAG', 'preempted':'FAILED', 'aborted':'FAILED'})

            smach.StateMachine.add('RIGHT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=rightMoveGoal),
                                   {'succeeded':'DRUM_NAVIGATION_LAG', 'preempted':'FAILED', 'aborted':'FAILED'})


            smach.StateMachine.add('DRUM_NAVIGATION_MARCH', march_direction_control(),
                                   transitions={'Y_PositionIsOK':'CENTERED',
                                                'NeedForwardMove':'FORWARD_MOVE',
                                                'NeedBackwardMove':'BACKWARDS_MOVE',
                                                'FAILED':'FAILED',
                                                'NO_DRUM_DETECTED':'WAITING_DRUM_DETECTION_MSG_MARCH'})
                                                #'NO_DRUM_DETECTED':'WAITING_DRUM_DETECTION_MSG'})

            # MonitorState outcome switches from valid to invalid
            smach.StateMachine.add('WAITING_DRUM_DETECTION_MSG_MARCH',
                                   smach_ros.MonitorState(
                                       '/drums/drum',
                                       DrumsCoordinates,
                                       drum_check),
                                   {'invalid':'DRUM_NAVIGATION_MARCH', 'valid':'WAITING_DRUM_DETECTION_MSG_MARCH', 'preempted':'FAILED'})

            #smach.StateMachine.add('SLEEP_MARCH', sleep_control(),
                                   #transitions={'SUCCESS':'DRUM_NAVIGATION_MARCH'})

            smach.StateMachine.add('FORWARD_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=forwardMoveGoal),
                                   {'succeeded':'DRUM_NAVIGATION_MARCH', 'preempted':'FAILED', 'aborted':'FAILED'})

            smach.StateMachine.add('BACKWARDS_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=backwardsMoveGoal),
                                   {'succeeded':'DRUM_NAVIGATION_MARCH', 'preempted':'FAILED', 'aborted':'FAILED'})

        smach.StateMachine.add('DRUM_CENTERING', sm_sub,
                                transitions={'CENTERED':'DRUMS_NAVIGATION_OK',
                                             'FAILED':'DRUMS_NAVIGATION_FAILED'})

    return sm