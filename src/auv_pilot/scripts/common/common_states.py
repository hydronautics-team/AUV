#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.srv import EnablingCmd
from auv_common.msg import DiveGoal, DiveAction, MoveAction, MoveGoal

# TODO: Create more common states

# Creates delay (in seconds)
class WaitState(smach.State):
    def __init__(self, delay):
        smach.State.__init__(self, outcomes=['OK'])
        self.delay = delay
        
    def execute(self, userdata):
        if self.delay == 0:
            return 'OK'
        rate = rospy.Rate(1.0 / self.delay)
        rate.sleep()
        return 'OK'

class IMUInitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OK'])

    def execute(self, userdata):
        imu_init_service = rospy.ServiceProxy("imu_init_service", EnablingCmd)
        imu_init_service(True)
        rospy.sleep(0.8)
        imu_init_service(False)
        return 'OK'


class StabilizationInitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OK'])

    def execute(self, userdata):
        stabilization_service = rospy.ServiceProxy("stabilization_service", EnablingCmd)
        stabilization_service(False)
        rospy.sleep(1.0)
        stabilization_service(True)
        rospy.sleep(5.0)
        return 'OK'

def create_timer_state(time):
    class mat_timer_fsm(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['ERROR'])
            self.timer_flag = True
            #self.timer = rospy.ServiceProxy("timer_service", TimerFsm)
            #self.timerControl = self.timer(False)
            #print (self.timerControl.duration)
            print ("Timer FSM created with delay parameter: ", time)

        def execute(self, userdata):
            rospy.sleep(time) # Test
            return 'ERROR'
    return mat_timer_fsm()

def create_time_calculation_state(time, sleep_time):
    class time_calculation(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['CALCULATING_TIME', 'TERMINATING'])
            self.state_time = rospy.get_rostime() # get time as rospy.Time instance
            self.timerFlag = True
            self.start_time = -1
            self.current_time = -1
            rospy.loginfo("Calculation state added with time parameter: %i", time)

        def execute(self, userdata):
            self.current_time = rospy.get_rostime() # get time as rospy.Time instance
            if self.timerFlag:
                self.start_time = rospy.get_rostime() # get time as rospy.Time instance
                rospy.loginfo("Start time %i %i", self.start_time.secs, self.start_time.nsecs)
                self.timerFlag = False
                rospy.sleep(sleep_time) # Test
                return 'CALCULATING_TIME'

            #elif reset:
                #self.timerFlag = True
                #rospy.loginfo("RESET")
                #return 'RESET'

            elif abs(self.current_time.secs - self.start_time.secs) > time:
                return 'TERMINATING'
            else:
                rospy.sleep(sleep_time) # Test
                return 'CALCULATING_TIME'
    return time_calculation()

# Creates diving state (depth in centimeters)
def create_diving_state(depth):
    dive = DiveGoal()
    dive.depth = depth
    return smach_ros.SimpleActionState('dive', DiveAction, goal=dive)

def create_signal_state():
    goal = MoveGoal()
    goal.direction = MoveGoal.DIRECTION_FORWARD
    goal.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
    goal.value = 1000
    return smach_ros.SimpleActionState('move_by_time', MoveAction, goal=goal)


def create_move_state(direction, value, velocity_level, hold_if_infinity=False):
    action_goal = MoveGoal()
    action_goal.direction = direction
    action_goal.velocityLevel = velocity_level
    action_goal.value = value
    action_goal.holdIfInfinityValue = hold_if_infinity
    return smach_ros.SimpleActionState('move_by_time', MoveAction, goal=action_goal)
