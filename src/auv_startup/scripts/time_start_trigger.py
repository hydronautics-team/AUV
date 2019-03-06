#! /usr/bin/env python

import rospy
import std_srvs.srv


def main():
    rospy.init_node('time_start_trigger')
    delay = int(rospy.get_param('delay', 5))
    rospy.sleep(delay)
    startup = rospy.ServiceProxy('controller_launch', std_srvs.srv.Trigger)
    startup()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass