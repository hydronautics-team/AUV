#! /usr/bin/env python

import roslib
roslib.load_manifest('auv_pilot')
import rospy
import actionlib

from auv_common import MoveByTimeAction

class MoveByTimeServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('MoveByTime', MoveByTimeAction, self.execute, False)
        self.server.start()
    def execute(self, goal):
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('pilot')
    server = MoveByTimeServer()
    rospy.spin()