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

    class edge_check(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['NO_EDGE_DETECTED', 'EDGE_DETECTED', 'FAILED'],
                                       output_keys=['MISSION_START_TIME'])

            self.subscriber = rospy.Subscriber('/drums/mat/cam_bottom', DistancesToMatEdges, self.callback)
            self.hasHorizontalEdge = False
            self.mission_start_time = -1
            self.flag = True

        def callback(self, matMessage):
            if matMessage.hasHorizontalLine:
                self.hasHorizontalEdge = True
            else:
                self.hasHorizontalEdge = False

        def execute(self, userdata):
            if self.flag:
                self.mission_start_time = rospy.get_rostime() # get time as rospy.Time instance
                rospy.loginfo("Mission start time %i %i", self.mission_start_time.secs, self.mission_start_time.nsecs)
                self.flag = False
                userdata.MISSION_START_TIME = self.mission_start_time
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

    '''
    class mat_detection(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['SHORT_TIME', 'LONG_TIME', 'FAILED'])
            self.state_time = -1

        def execute(self, userdata):
            print ("DELTA")
            print (abs(start_time - rospy.get_rostime()))
            if abs(start_time - rospy.get_rostime()) < 1000:
                return 'SHORT_TIME'
            else:
                return 'LONG_TIME'
    '''

    class time_calculation(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['CALCULATING_TIME', 'TERMINATING'],
                                       input_keys=['TIME'])
            self.state_time = rospy.get_rostime() # get time as rospy.Time instance
            self.timerFlag = True
            self.start_time = -1
            self.current_time = -1

        def execute(self, userdata):
            self.current_time = rospy.get_rostime() # get time as rospy.Time instance
            if self.timerFlag:
                self.start_time = rospy.get_rostime() # get time as rospy.Time instance
                rospy.loginfo("Start time %i %i", self.start_time.secs, self.start_time.nsecs)
                self.timerFlag = False
                return 'CALCULATING_TIME'
            elif abs(self.current_time.secs - self.start_time.secs) > 50 and userdata.TIME.secs > 3*60:
                print ("userdata.TIME.secs")
                print (userdata.TIME.secs)
                print ("self.start_time.secs")
                print (self.start_time.secs)
                return 'TERMINATING'
            else:
                return 'CALCULATING_TIME'

    def mat_check(userData, matMessage):
        return not matMessage.hasPoint

    # gets called when ANY child state terminates
    def child_term_cb(outcome_map):
        # terminate all running states if we received msg with mat coordinates
        if outcome_map['WAITING_MAT_DETECTION_MSG'] == 'invalid':
            return True
        # terminate all running states if a lot of time passed
        if outcome_map['TIMER'] == 'TERMINATING':
            return True
        if outcome_map['TIMER'] == 'CALCULATING_TIME':
            return True
        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated
    def out_cb(outcome_map):
        if outcome_map['WAITING_MAT_DETECTION_MSG'] == 'invalid': # we received msg with mat coordinates
            return 'MatDetectedUsingFrontCamera'
        if outcome_map['TIMER'] == 'TERMINATING':
            return 'LongTimeNoEdge'
        return 'CalculatingTime'

    # creating the concurrence state machine
    concurrence_state = smach.Concurrence(outcomes=['MatDetectedUsingFrontCamera', 'LongTimeNoEdge', 'CalculatingTime'],
                                          default_outcome='CalculatingTime', # if none of the mappings are satisfied, the concurrence will return its default outcome
                                          input_keys=['MISSION_START_TIME'],
                                          child_termination_cb = child_term_cb,
                                          outcome_cb = out_cb)

    with concurrence_state:
        smach.Concurrence.add('TIMER', time_calculation(),
                              remapping={'TIME':'MISSION_START_TIME'})
        smach.Concurrence.add('WAITING_MAT_DETECTION_MSG',
                              smach_ros.MonitorState(
                                  '/drums/mat/cam_front',
                                  OptionalPoint2D,
                                  mat_check))


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
        forwardMoveGoal.value = 2000
        forwardMoveGoal.holdIfInfinityValue = False

        forwardMoveAboveMatGoal = MoveGoal()
        forwardMoveAboveMatGoal.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveAboveMatGoal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardMoveAboveMatGoal.value = 800
        forwardMoveAboveMatGoal.holdIfInfinityValue = False


        smach.StateMachine.add('FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardMoveGoal),
                               {'succeeded':'MAT_HORIZONTAL_EDGE_CHECK', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED', 'aborted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('MAT_HORIZONTAL_EDGE_CHECK', edge_check(),
                                transitions={'NO_EDGE_DETECTED':'MAT_CENTERING',
                                             'EDGE_DETECTED':'HORIZONTAL_EDGE_DETECTED',
                                             'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'},
                                remapping={'MISSION_START_TIME':'sm_time'})

        '''
        smach.StateMachine.add('MAT_DETECTION', mat_detection(),
                               transitions={'SHORT_TIME':'WAITING_MAT_DETECTION_MSG',
                                            'LONG_TIME':'FORWARD_MOVE_ABOVE_MAT',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})
        '''

        smach.StateMachine.add('MAT_DETECTION',
                               concurrence_state,
                               transitions={'MatDetectedUsingFrontCamera':'MAT_CENTERING',
                                            'LongTimeNoEdge':'FORWARD_MOVE_ABOVE_MAT',
                                            'CalculatingTime':'CIRCLE'},
                               remapping={'MISSION_START_TIME':'sm_time'})

        smach.StateMachine.add('CIRCLE', time_calculation(),
                               transitions={'CALCULATING_TIME':'MAT_DETECTION', 'TERMINATING':'FORWARD_MOVE_ABOVE_MAT'},
                               remapping={'TIME':'sm_time'})

        smach.StateMachine.add('FORWARD_MOVE_ABOVE_MAT',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardMoveAboveMatGoal),
                               {'succeeded':'MAT_HORIZONTAL_EDGE_CHECK_ABOVE_MAT', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED', 'aborted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('MAT_HORIZONTAL_EDGE_CHECK_ABOVE_MAT', edge_check_above_mat(),
                               transitions={'NO_EDGE_DETECTED':'FORWARD_MOVE_ABOVE_MAT',
                                            'EDGE_DETECTED':'HORIZONTAL_EDGE_DETECTED',
                                            'MAT_DETECTED':'FORWARD_MOVE',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        # MonitorState outcome switches from valid to invalid
        #smach.StateMachine.add('WAITING_MAT_DETECTION_MSG',
        #                       smach_ros.MonitorState(
         #                          '/drums/mat/cam_front',
          #                          OptionalPoint2D,
           #                        mat_check),
           #                    {'invalid':'MAT_CENTERING', 'valid':'WAITING_MAT_DETECTION_MSG', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

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
                                   {'succeeded':'MAT_NAVIGATION_LAG', 'preempted':'FAILED', 'aborted':'FAILED'})

            smach.StateMachine.add('RIGHT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=rightMoveGoal),
                                   {'succeeded':'MAT_NAVIGATION_LAG', 'preempted':'FAILED', 'aborted':'FAILED'})


        smach.StateMachine.add('MAT_CENTERING', sm_sub_mat,
                                transitions={'CENTERED':'FORWARD_MOVE',
                                             'DETECTING_MAT':'MAT_DETECTION',
                                             'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})


    return sm