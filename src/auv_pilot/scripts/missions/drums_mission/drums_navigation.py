#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import DrumsCoordinates
from auv_common.msg import MoveGoal, MoveAction


# TODO: Try to implement through extending smach.StateMachine class
def create_drums_navigation_fsm():


    class lag_direction_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['X_PositionIsOK', 'NeedRightMove', 'NeedLeftMove', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/drum', DrumsCoordinates, self.callback)
            self.lagMoveNeeded = False
            self.rightMove = False
            self.leftMove = False

        def callback(self, drumMessage):
            if abs(drumMessage.RedDrum1x) > 40:
                self.lagMoveNeeded = True
                if drumMessage.RedDrum1x > 0:
                    self.rightMove = True
                else:
                    self.leftMove = True

        def execute(self, userdata):
            print (self.lagMoveNeeded)
            print (self.rightMove)
            if self.lagMoveNeeded:
                if self.rightMove:
                    return 'NeedRightMove'
                if self.leftMove:
                    return 'NeedLeftMove'
            else:
                return 'X_PositionIsOK'


    class march_direction_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['Y_PositionIsOK', 'NeedForwardMove', 'NeedBackwardMove', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/drum', DrumsCoordinates, self.callback)
            self.marchMoveNeeded = False
            self.forwardMove = False
            self.backwardMove = False

        def callback(self, drumMessage):
            if abs(drumMessage.RedDrum1y) > 40:
                self.marchMoveNeeded = True
                if drumMessage.RedDrum1y > 0:
                    self.forwardMove = True
                else:
                    self.backwardMove = True

        def execute(self, userdata):
            if self.marchMoveNeeded:
                if self.forwardMove:
                    return 'NeedForwardMove'
                if self.backwardMove:
                    return 'NeedBackwardMove'
            else:
                return 'Y_PositionIsOK'



    sm = smach.StateMachine(outcomes=['DRUMS_NAVIGATION_OK', 'DRUMS_NAVIGATION_FAILED'])

    with sm:

        leftMoveGoal = MoveGoal()
        leftMoveGoal.direction = MoveGoal.DIRECTION_LEFT
        leftMoveGoal.value = 800
        leftMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        leftMoveGoal.holdIfInfinityValue = False

        rightMoveGoal = MoveGoal()
        rightMoveGoal.direction = MoveGoal.DIRECTION_RIGHT
        rightMoveGoal.value = 800
        rightMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        rightMoveGoal.holdIfInfinityValue = False


        forwardMoveGoal = MoveGoal()
        forwardMoveGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardMoveGoal.value = 800
        forwardMoveGoal.holdIfInfinityValue = False

        backwardsMoveGoal = MoveGoal()
        backwardsMoveGoal.direction = MoveGoal.DIRECTION_BACKWARDS
        backwardsMoveGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        backwardsMoveGoal.value = 800
        backwardsMoveGoal.holdIfInfinityValue = False


        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['CENTERED', 'FAILED'])

        with sm_sub:
            smach.StateMachine.add('DRUM_NAVIGATION_LAG', lag_direction_control(),
                                   transitions={'X_PositionIsOK':'DRUM_NAVIGATION_MARCH',
                                                'NeedLeftMove':'LEFT_MOVE',
                                                'NeedRightMove':'RIGHT_MOVE',
                                                'FAILED':'FAILED'})

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
                                                'FAILED':'FAILED'})

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