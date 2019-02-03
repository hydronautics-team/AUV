#! /usr/bin/env python

import rospy
import smach


class SimpleInitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OK'])
        
    def execute(self, userdata):
        # We have to wait our action servers to initialize
        rate = rospy.Rate(1)
        rate.sleep()
        return 'OK'