#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import DrumsCoordinates, DistancesToMatEdges
from auv_common.msg import MoveGoal, MoveAction


# TODO: Try to implement through extending smach.StateMachine class
def create_mat_bottom_cam_navigation_fsm():

    distance = rospy.get_param('~drum_mission_horizontal_edge_distance', 0)
    rospy.loginfo("FSM drums_mission distance: ")
    rospy.loginfo(distance)

    class horizontal_edge_position(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['OK', 'NeedForwardMove', 'NeedBackwardMove', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/mat/cam_bottom', DistancesToMatEdges, self.callback)
            self.correctionNeeded = False
            self.forwardMove = False
            self.backwardMove = False
            self.delta = 15 # Tolerance parameter

        def callback(self, matMessage):
            if matMessage.distanceToHorizontalLine < distance - self.delta or matMessage.distanceToHorizontalLine > distance + self.delta:
                self.correctionNeeded = True
                if matMessage.distanceToHorizontalLine < distance - self.delta:
                    self.forwardMove = True
                    self.backwardMove = False
                else:
                    self.backwardMove = True
                    self.forwardMove = False
            else: self.correctionNeeded = False

        def execute(self, userdata):
            if self.correctionNeeded:
                if self.forwardMove:
                    return 'NeedForwardMove'
                if self.backwardMove:
                    return 'NeedBackwardMove'
            else:
                return 'OK'

    class drum_detection_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['RED_DRUM_DETECTED', 'BLUE_DRUM_DETECTED', 'NO_DRUM_DETECTED', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/drum', DrumsCoordinates, self.callback)
            self.redDetected = False
            self.blueDetected = False

        def callback(self, drumMessage):
            if drumMessage.hasRedDrum and drumMessage.hasBlueDrum:
                self.redDetected = True
                self.blueDetected = True
            elif drumMessage.hasBlueDrum:
                self.redDetected = False
                self.blueDetected = True
            elif drumMessage.hasRedDrum:
                self.redDetected = True
                self.blueDetected = False
            else:
                self.redDetected = False
                self.blueDetected = False

        def execute(self, userdata):
            if self.redDetected and self.blueDetected:
                return 'BLUE_DRUM_DETECTED'
            elif self.redDetected:
                'RED_DRUM_DETECTED'
            elif self.blueDetected:
                'BLUE_DRUM_DETECTED'
            else:
                return 'NO_DRUM_DETECTED'

    class vertical_edge_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['VERTICAL_EDGE_DETECTED', 'NO_VERTICAL_EDGE_DETECTED', 'NEED_TO_REVERSE', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/mat/cam_bottom', DistancesToMatEdges, self.callback)
            self.detected = False
            self.delta = 20 # Tolerance parameter
            self.needToReverse = False

        def callback(self, matMessage):
            if matMessage.hasVerticalLine:
                self.detected = True
                if abs(matMessage.distanceToVerticalLine) < self.delta:
                    self.needToReverse = True
                else:
                    self.needToReverse = False
            else:
                self.detected = False

        def execute(self, userdata):
            if self.detected:
                if self.needToReverse:
                    return 'NEED_TO_REVERSE'
                else:
                    return 'VERTICAL_EDGE_DETECTED'
            else:
                return 'NO_VERTICAL_EDGE_DETECTED'

    class reverse_direction(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['REVERSED', 'FAILED'],
                                       output_keys=['CONDITION', 'COUNTER'])
            self.switch = True # Means LEFT
            self.counter = 0

        def execute(self, userdata):
            self.switch = not self.switch
            userdata.CONDITION = self.switch
            self.counter += 1
            userdata.COUNTER = self.counter
            return 'REVERSED'

    class choose_direction(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['LEFT_MOVE', 'RIGHT_MOVE', 'FAILED'],
                                       input_keys=['CONDITION'])
        def execute(self, userdata):
            print (userdata.CONDITION)
            if userdata.CONDITION:
                return 'LEFT_MOVE'
            else:
                return 'RIGHT_MOVE'

    class error_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['OK', 'SOMETHING_WRONG', 'FAILED'],
                                       input_keys=['COUNTER'],
                                       output_keys=['FLAG'])
        def execute(self, userdata):
            if userdata.COUNTER >= 2:
                userdata.FLAG = True
                return 'SOMETHING_WRONG'
            else:
                userdata.FLAG = False
                return 'OK'

    class error_flag_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['NO_ERROR', 'ERROR', 'FAILED'],
                                       input_keys=['FLAG'])
        def execute(self, userdata):
            if not userdata.FLAG:
                return 'NO_ERROR'
            else:
                return 'ERROR'

    sm = smach.StateMachine(outcomes=['BLUE_DRUM_DETECTED', 'RED_DRUM_DETECTED', 'MAT_BOTTOM_CAM_NAVIGATION_FAILED'])

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


        smach.StateMachine.add('FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardMoveGoal),
                               {'succeeded':'HORIZONTAL_EDGE_POSITION', 'preempted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED', 'aborted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('BACKWARDS_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=backwardsMoveGoal),
                               {'succeeded':'HORIZONTAL_EDGE_POSITION', 'preempted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED', 'aborted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('HORIZONTAL_EDGE_POSITION', horizontal_edge_position(),
                              transitions={'OK':'MOVE',
                                           'NeedForwardMove':'FORWARD_MOVE',
                                           'NeedBackwardMove':'BACKWARDS_MOVE',
                                           'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('DRUM_DETECTION_CHECK', drum_detection_check(),
                               transitions={'RED_DRUM_DETECTED':'ERROR_FLAG_CHECK',
                                            'BLUE_DRUM_DETECTED':'BLUE_DRUM_DETECTED',
                                            'NO_DRUM_DETECTED':'VERTICAL_EDGE_CHECK',
                                            'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('ERROR_FLAG_CHECK', error_flag_check(),
                               transitions={'NO_ERROR':'VERTICAL_EDGE_CHECK',
                                            'ERROR':'RED_DRUM_DETECTED',
                                            'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'},
                               remapping={'FLAG':'sm_flag'})

        smach.StateMachine.add('VERTICAL_EDGE_CHECK', vertical_edge_check(),
                              transitions={'NEED_TO_REVERSE':'REVERSE_DIRECTION',
                                           'VERTICAL_EDGE_DETECTED':'HORIZONTAL_EDGE_POSITION',
                                           'NO_VERTICAL_EDGE_DETECTED':'HORIZONTAL_EDGE_POSITION',
                                           'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('REVERSE_DIRECTION', reverse_direction(),
                              transitions={'REVERSED':'ERROR_CHECK',
                                           'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'},
                              remapping={'CONDITION':'sm_condition',
                                         'COUNTER':'sm_counter'})

        smach.StateMachine.add('MOVE', choose_direction(),
                              transitions={'LEFT_MOVE':'LEFT_MOVE',
                                           'RIGHT_MOVE':'RIGHT_MOVE',
                                           'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'},
                              remapping={'CONDITION':'sm_condition'})

        # Check if we have swum above the mat several times and didn't detect drums
        smach.StateMachine.add('ERROR_CHECK', error_check(),
                              transitions={'OK':'MOVE',
                                           'SOMETHING_WRONG':'MOVE',
                                           'FAILED':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'},
                              remapping={'COUNTER':'sm_counter',
                                         'FLAG':'sm_flag'})

        smach.StateMachine.add('LEFT_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=leftMoveGoal),
                               {'succeeded':'DRUM_DETECTION_CHECK', 'preempted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED', 'aborted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('RIGHT_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=rightMoveGoal),
                               {'succeeded':'DRUM_DETECTION_CHECK', 'preempted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED', 'aborted':'MAT_BOTTOM_CAM_NAVIGATION_FAILED'})


    return sm