#! /usr/bin/env python

import rospy
import roslaunch
import std_msgs.msg
import std_srvs.srv


launched = False
requestedMode = None
launchRequested = False
stopRequested = False


def barcode_callback(msg):
    global launched
    global launchRequested
    global stopRequested
    global requestedMode

    message = msg.data.lower()
    if message not in ['qualification_simple', 'qualification_vision', 'missions', 'demo', 'stop']:
        rospy.logerr('Unknown messages')
        return

    if message == 'stop':
        if not launched:
            rospy.logwarn('Not launched yet')
            return
        if stopRequested:
            rospy.logwarn('Stop is already requested')
            return
        stopRequested = True
        return

    if launched:
        rospy.logwarn('Already launched')
        return
    if launchRequested:
        rospy.logwarn('Launch is already requested')
        return

    requestedMode = message
    launchRequested = True


def stop_notification_callback(req):
    global stopRequested
    stopRequested = True
    return std_srvs.srv.TriggerResponse(True, 'OK')

def main():
    global launched
    global launchRequested
    global stopRequested
    global requestedMode

    rospy.init_node('qr_trigger')

    launch = None

    rospy.Subscriber('/barcode', std_msgs.msg.String, barcode_callback)
    rospy.Service('global_fsm_finished', std_srvs.srv.Trigger, stop_notification_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if launchRequested:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/sibirsky/factory/AUV/src/auv_startup/launch/" +
                                                     requestedMode + ".launch"])
            rospy.loginfo('Starting launch mode ' + requestedMode)
            launch.start()
            launched = True
            launchRequested = False
        elif stopRequested:
            rospy.loginfo('Shutting down launch config...')
            launch.shutdown()
            launched = False
            stopRequested = False

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
