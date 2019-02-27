#! /usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from auv_common.msg import OptionalPoint2D, DistancesToMatEdges
from auv_common.msg import MoveGoal, MoveAction


# TODO: Try to implement through extending smach.StateMachine class
def create_mat_front_cam_navigation_fsm():

    class lag_direction_control(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['X_PositionIsOK', 'NeedRightMove', 'NeedLeftMove', 'MatPositionLost'])

            self.subscriber = rospy.Subscriber('/drums/mat/cam_front', OptionalPoint2D, self.callback)
            self.hasPoint = False
            self.lagMoveNeeded = False
            self.rightMove = False
            self.leftMove = False

        def callback(self, matMessage):
            if matMessage.hasPoint:
                self.hasPoint = True
                if abs(matMessage.x) > 40:
                    self.lagMoveNeeded = True
                    if matMessage.x > 0:
                        self.rightMove = True
                        self.leftMove = False
                    else:
                        self.leftMove = True
                        self.rightMove = False
                else:
                    self.lagMoveNeeded = False
            else:
                self.hasPoint = False

        def execute(self, userdata):
            if self.hasPoint:
                if self.lagMoveNeeded:
                    if self.rightMove:
                        return 'NeedRightMove'
                    if self.leftMove:
                        return 'NeedLeftMove'
                else:
                    return 'X_PositionIsOK'
            else:
                return 'MatPositionLost'

    def mat_check(userData, matMessage):
        return not matMessage.hasPoint

    class edge_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['NO_EDGE_DETECTED', 'EDGE_DETECTED', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/mat/cam_bottom', DistancesToMatEdges, self.callback)
            self.hasHorizontalEdge = False

        def callback(self, matMessage):
            if matMessage.hasHorizontalLine:
                self.hasHorizontalEdge = True
            else:
                self.hasHorizontalEdge = False

        def execute(self, userdata):
            if self.hasHorizontalEdge:
                return 'EDGE_DETECTED'
            else:
                return 'NO_EDGE_DETECTED'

    class edge_check_above_mat(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['NO_EDGE_DETECTED', 'EDGE_DETECTED', 'MAT_DETECTED', 'FAILED'])

            self.subscriber_bottom = rospy.Subscriber('/drums/mat/cam_bottom', DistancesToMatEdges, self.callback_bottom)
            self.subscriber_front = rospy.Subscriber('/drums/mat/cam_front', OptionalPoint2D, self.callback_front)
            self.hasHorizontalEdge = False
            self.hasPoint = False

        def callback_bottom(self, matMessage):
            if matMessage.hasHorizontalLine:
                self.hasHorizontalEdge = True
            else:
                self.hasHorizontalEdge = False

        def callback_front(self, matMessage):
            if matMessage.hasPoint:
                self.hasPoint = True
            else:
                self.hasPoint = False

        def execute(self, userdata):
            if self.hasHorizontalEdge:
                return 'EDGE_DETECTED'
            elif self.hasPoint:
                return 'MAT_DETECTED'
            else:
                return 'NO_EDGE_DETECTED'

    class mat_detection(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['SHORT_TIME', 'LONG_TIME', 'FAILED'])

        def execute(self, userdata):
            print ("DELTA")
            print (abs(start_time - rospy.get_rostime()))
            if abs(start_time - rospy.get_rostime()) < 1000:
                return 'SHORT_TIME'
            else:
                return 'LONG_TIME'

    start_time = rospy.get_rostime() # get time as rospy.Time instance
    print ("start_time")
    print (start_time)

    sm = smach.StateMachine(outcomes=['HORIZONTAL_EDGE_DETECTED', 'MAT_FRONT_CAM_NAVIGATION_FAILED'])

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


        smach.StateMachine.add('FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardMoveGoal),
                               {'succeeded':'MAT_HORIZONTAL_EDGE_CHECK', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED', 'aborted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('MAT_HORIZONTAL_EDGE_CHECK', edge_check(),
                                transitions={'NO_EDGE_DETECTED':'MAT_CENTERING',
                                             'EDGE_DETECTED':'HORIZONTAL_EDGE_DETECTED',
                                             'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('MAT_DETECTION', mat_detection(),
                               transitions={'SHORT_TIME':'WAITING_MAT_DETECTION_MSG',
                                            'LONG_TIME':'FORWARD_MOVE_ABOVE_MAT',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('FORWARD_MOVE_ABOVE_MAT',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardMoveGoal),
                               {'succeeded':'MAT_HORIZONTAL_EDGE_CHECK_ABOVE_MAT', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED', 'aborted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('MAT_HORIZONTAL_EDGE_CHECK_ABOVE_MAT', edge_check_above_mat(),
                               transitions={'NO_EDGE_DETECTED':'FORWARD_MOVE_ABOVE_MAT',
                                            'EDGE_DETECTED':'HORIZONTAL_EDGE_DETECTED',
                                            'MAT_DETECTED':'FORWARD_MOVE',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        # MonitorState outcome switches from valid to invalid
        smach.StateMachine.add('WAITING_MAT_DETECTION_MSG',
                               smach_ros.MonitorState(
                                   '/drums/mat/cam_front',
                                    OptionalPoint2D,
                                    mat_check),
                               {'invalid':'MAT_CENTERING', 'valid':'WAITING_MAT_DETECTION_MSG', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        # Create the sub SMACH state machine
        sm_sub_mat = smach.StateMachine(outcomes=['CENTERED', 'DETECTING_MAT', 'FAILED'])

        with sm_sub_mat:
            smach.StateMachine.add('MAT_NAVIGATION_LAG', lag_direction_control(),
                                   transitions={'X_PositionIsOK':'CENTERED',
                                                'NeedLeftMove':'LEFT_MOVE',
                                                'NeedRightMove':'RIGHT_MOVE',
                                                'MatPositionLost':'DETECTING_MAT'})

            smach.StateMachine.add('LEFT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=leftMoveGoal),
                                   {'succeeded':'CENTERED', 'preempted':'FAILED', 'aborted':'FAILED'})

            smach.StateMachine.add('RIGHT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=rightMoveGoal),
                                   {'succeeded':'CENTERED', 'preempted':'FAILED', 'aborted':'FAILED'})


        smach.StateMachine.add('MAT_CENTERING', sm_sub_mat,
                                transitions={'CENTERED':'FORWARD_MOVE',
                                             'DETECTING_MAT':'MAT_DETECTION',
                                             'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})


    return sm