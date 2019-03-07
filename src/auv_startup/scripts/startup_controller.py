#! /usr/bin/env python

import rospy
import roslaunch
import std_srvs.srv
import os

launchRequested = False
launched = False
shutDownRequested = False


def lauch_callback(req):
    global launchRequested
    global launched
    if launchRequested:
        resp = std_srvs.srv.TriggerResponse
        resp.success = False
        resp.message = "Launch is already requested"
        return resp
    if launched:
        resp = std_srvs.srv.TriggerResponse
        resp.success = False
        resp.message = "Already launched"
        return resp
    launchRequested = True


def stop_callback(req):
    global shutDownRequested
    global launched
    if shutDownRequested:
        resp = std_srvs.srv.TriggerResponse
        resp.success = False
        resp.message = "Shutdown is already requested"
        return resp
    if not launched:
        resp = std_srvs.srv.TriggerResponse
        resp.success = False
        resp.message = "Not launched yet"
        return resp
    shutDownRequested = True


def poweroff_callback(req):
    # TODO: stop launch file
    rospy.logwarn('Poweroff request handled, shutting down machine')
    os.system('shutdown now')


def main():
    global launchRequested
    global launched
    global shutDownRequested

    rospy.init_node('startup_controller')

    launch_service = rospy.Service('controller_launch', std_srvs.srv.Trigger, lauch_callback)
    stop_service = rospy.Service('controller_stop', std_srvs.srv.Trigger, stop_callback)
    poweroff_service = rospy.Service('controller_poweroff', std_srvs.srv.Trigger, poweroff_callback)

    launch = None

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if launchRequested and not launched:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/sibirsky/factory/AUV/launch/AUV.launch"])
            rospy.loginfo("Launching...")
            launch.start()
            launched = True
            launchRequested = False
        elif shutDownRequested and launched:
            rospy.loginfo("Stopping launch config...")
            launch.shutdown()
            launched = False
            shutDownRequested = False
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass