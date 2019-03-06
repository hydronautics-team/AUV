#! /usr/bin/env python

import rospy
import smach
import smach_ros
from auv_common.srv import *
from auv_common.msg import DiveGoal, DiveAction

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


class IMUResetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OK'])

    def execute(self, userdata):
        reset_service = rospy.ServiceProxy("reset_service", ResetCmd)
        response = reset_service()
        rospy.sleep(0.5)
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

# Creates diving state (depth in centimeters)
def create_diving_state(depth):
    dive = DiveGoal()
    dive.depth = depth
    return smach_ros.SimpleActionState('dive', DiveAction, goal=dive)

