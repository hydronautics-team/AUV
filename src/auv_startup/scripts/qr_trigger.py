#! /usr/bin/env python

import rospy
import roslaunch
import os
import std_msgs.msg
import auv_common.srv

launched = False

def barcode_callback(msg):
    global launched

    message = msg.data.lower()
    if message not in ['qualification_simple', 'qualification_vision', 'missions', 'shutdown']:
        rospy.logerr('Unknown messages')
        return

    if message == 'shutdown':
        rospy.loginfo('Shutting down machine...')
        os.system('shutdown now')
        return

    if launched:
        rospy.logwarn('Already launched')
        return

    launch_service = rospy.ServiceProxy('fsm_start', auv_common.srv.LaunchCmd)
    launch_service(message)
    launched = True


def main():
    rospy.init_node('qr_trigger')

    rospy.Subscriber('/barcode', std_msgs.msg.String, barcode_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
