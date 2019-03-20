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

    '''
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
            smach.State.__init__(self, outcomes=['SUCCESS'])
            #self.subscriber = rospy.Subscriber('/drums/mat/cam_front', OptionalPoint2D, self.callback)

        def execute(self, userdata):
            # sleep for 10 seconds
            rospy.sleep(1.)
            return 'SUCCESS'


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
                rospy.sleep(0.7) # Test
                return 'CALCULATING_TIME'
            elif abs(self.current_time.secs - self.start_time.secs) > 50: # and userdata.TIME.secs > 3*60
                return 'TERMINATING'
            else:
                rospy.sleep(0.7) # Test
                return 'CALCULATING_TIME'


    class detecting_drum_front_cam(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['DRUM_DETECTED', 'NO_DRUM_DETECTED', 'FAILED'])

            self.subscriber = rospy.Subscriber('/drums/cam_front', OptionalPoint2D, self.callback)
            self.drumDetected = False

        def callback(self, drumMessage):
            if drumMessage.hasPoint:
                self.drumDetected = True
            else: self.drumDetected = False

        def execute(self, userdata):
            if self.drumDetected:
                return 'DRUM_DETECTED'
            else:
                return 'NO_DRUM_DETECTED'


    class lag_direction_control_drum(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['X_PositionIsOK', 'NeedRightMove', 'NeedLeftMove', 'DrumPositionLost'])

            self.subscriber = rospy.Subscriber('/drums/cam_front', OptionalPoint2D, self.callback)
            self.hasPoint = False
            self.lagMoveNeeded = False
            self.rightMove = False
            self.leftMove = False
            self.counter = 0

        def callback(self, matMessage):
            if matMessage.hasPoint:
                self.hasPoint = True
                if abs(matMessage.x) > 40:
                    self.lagMoveNeeded = True
                    self.counter += 1
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
            if self.counter < 10:
                if self.hasPoint:
                    if self.lagMoveNeeded:
                        if self.rightMove:
                            return 'NeedRightMove'
                        if self.leftMove:
                            return 'NeedLeftMove'
                    else:
                        return 'X_PositionIsOK'
                else:
                    return 'DrumPositionLost'
            else:
                'X_PositionIsOK'


    '''
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

                                          child_termination_cb = child_term_cb,
                                          outcome_cb = out_cb)

    with concurrence_state:
        smach.Concurrence.add('TIMER', time_calculation(),
                              remapping={'TIME':'MISSION_START_TIME'})



        smach.Concurrence.add('WAITING_DRUM_DETECTION_MSG',
                              smach_ros.MonitorState(
                                  '/drums/drum_front',
                                  OptionalPoint2D,
                                  drum_check))
    '''

    def mat_check(userData, matMessage):
        return not matMessage.hasPoint

    def drum_check(userData, matMessage):
        return not matMessage.hasPoint

    sm = smach.StateMachine(outcomes=['DRUM_DETECTED', 'MAT_FRONT_CAM_NAVIGATION_FAILED'])

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
        forwardMoveGoal.value = 1000
        forwardMoveGoal.holdIfInfinityValue = False


        leftMoveGoalDrum = MoveGoal()
        leftMoveGoalDrum.direction = MoveGoal.DIRECTION_LEFT
        leftMoveGoalDrum.value = 500
        leftMoveGoalDrum.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        leftMoveGoalDrum.holdIfInfinityValue = False

        rightMoveGoalDrum = MoveGoal()
        rightMoveGoalDrum.direction = MoveGoal.DIRECTION_RIGHT
        rightMoveGoalDrum.value = 500
        rightMoveGoalDrum.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        rightMoveGoalDrum.holdIfInfinityValue = False

        forwardMoveGoalDrum = MoveGoal()
        forwardMoveGoalDrum.direction = MoveGoal.DIRECTION_FORWARD
        forwardMoveGoalDrum.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
        forwardMoveGoalDrum.value = 1000
        forwardMoveGoalDrum.holdIfInfinityValue = False

        '''
        smach.StateMachine.add('MAT_HORIZONTAL_EDGE_CHECK', edge_check(),
                               transitions={'NO_EDGE_DETECTED':'MAT_CENTERING',
                                            'EDGE_DETECTED':'HORIZONTAL_EDGE_DETECTED',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'},
                               remapping={'MISSION_START_TIME':'sm_time'})
        '''

        # Create the sub SMACH state machine
        sm_sub_mat = smach.StateMachine(outcomes=['CENTERED', 'FAILED'])

        with sm_sub_mat:
            smach.StateMachine.add('MAT_NAVIGATION_LAG', lag_direction_control(),
                                   transitions={'X_PositionIsOK':'CENTERED',
                                                'NeedLeftMove':'LEFT_MOVE',
                                                'NeedRightMove':'RIGHT_MOVE',
                                                'MatPositionLost':'WAITING_MAT_DETECTION_MSG'})

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

            # MonitorState outcome switches from valid to invalid
            smach.StateMachine.add('WAITING_MAT_DETECTION_MSG',
                                   smach_ros.MonitorState(
                                       '/drums/mat/cam_front',
                                       OptionalPoint2D,
                                       mat_check),
                                   {'invalid':'MAT_NAVIGATION_LAG', 'valid':'WAITING_MAT_DETECTION_MSG', 'preempted':'FAILED'})


        smach.StateMachine.add('MAT_CENTERING', sm_sub_mat,
                                transitions={'CENTERED':'FORWARD_MOVE',
                                             'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})



        smach.StateMachine.add('FORWARD_MOVE',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forwardMoveGoal),
                               {'succeeded':'DRUM_DETECTION_FRONT_CAM', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED', 'aborted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})


        #smach.StateMachine.add('MAT_DETECTION', mat_detection(),
                               #transitions={'SUCCESS':'MAT_CENTERING'})


        smach.StateMachine.add('DRUM_DETECTION_FRONT_CAM', detecting_drum_front_cam(),
                               transitions={'DRUM_DETECTED':'DRUM_CENTERING', 'NO_DRUM_DETECTED':'LAG_MOVE_ADJUSTMENT', 'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        smach.StateMachine.add('LAG_MOVE_ADJUSTMENT',
                               smach_ros.SimpleActionState(
                                    'move_by_time',
                                    MoveAction,
                                    goal=rightMoveGoalDrum),
                               {'succeeded':'DRUM_DETECTION_FRONT_CAM', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED', 'aborted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        # Create the sub SMACH state machine
        sm_sub_drum = smach.StateMachine(outcomes=['CENTERED', 'FAILED'])

        with sm_sub_drum:
            smach.StateMachine.add('DRUM_NAVIGATION_LAG', lag_direction_control_drum(),
                                   transitions={'X_PositionIsOK':'CENTERED',
                                                'NeedLeftMove':'LEFT_MOVE',
                                                'NeedRightMove':'RIGHT_MOVE',
                                                'DrumPositionLost':'WAITING_DRUM_DETECTION_MSG'})

            # MonitorState outcome switches from valid to invalid
            smach.StateMachine.add('WAITING_DRUM_DETECTION_MSG',
                                   smach_ros.MonitorState(
                                       '/drums/cam_front',
                                       OptionalPoint2D,
                                       drum_check),
                                   {'invalid':'DRUM_NAVIGATION_LAG', 'valid':'WAITING_DRUM_DETECTION_MSG', 'preempted':'FAILED'})

            smach.StateMachine.add('LEFT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=leftMoveGoalDrum),
                                   {'succeeded':'DRUM_NAVIGATION_LAG', 'preempted':'FAILED', 'aborted':'FAILED'})

            smach.StateMachine.add('RIGHT_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=rightMoveGoalDrum),
                                   {'succeeded':'DRUM_NAVIGATION_LAG', 'preempted':'FAILED', 'aborted':'FAILED'})


        smach.StateMachine.add('DRUM_CENTERING', sm_sub_drum,
                               transitions={'CENTERED':'FORWARD_MOVE_UNTIL_DRUM_DETECTED',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        # Create the sub SMACH state machine
        sm_sub_drum_navigation = smach.StateMachine(outcomes=['DRUM_DETECTED', 'FAILED'])

        with sm_sub_drum_navigation:
            smach.StateMachine.add('FORWARD_MOVE',
                                   smach_ros.SimpleActionState(
                                       'move_by_time',
                                       MoveAction,
                                       goal=forwardMoveGoal),
                                   {'succeeded':'DRUM_DETECTION_BOTTOM_CAM', 'preempted':'FAILED', 'aborted':'FAILED'})

            smach.StateMachine.add('DRUM_DETECTION_BOTTOM_CAM', detecting_drum_front_cam(),
                                   transitions={'DRUM_DETECTED':'DRUM_DETECTED', 'NO_DRUM_DETECTED':'FORWARD_MOVE', 'FAILED':'FAILED'})

        smach.StateMachine.add('FORWARD_MOVE_UNTIL_DRUM_DETECTED', sm_sub_drum_navigation,
                               transitions={'DRUM_DETECTED':'DRUM_DETECTED',
                                            'FAILED':'MAT_FRONT_CAM_NAVIGATION_FAILED'})

        '''
        # MonitorState outcome switches from valid to invalid
        smach.StateMachine.add('WAITING_MAT_DETECTION_MSG',
                               smach_ros.MonitorState(
                                   '/drums/mat/cam_front',
                                    OptionalPoint2D,
                                   mat_check),
                               {'invalid':'MAT_CENTERING', 'valid':'WAITING_MAT_DETECTION_MSG', 'preempted':'MAT_FRONT_CAM_NAVIGATION_FAILED'})
        '''

    return sm