#! /usr/bin/env python

import rospy
import roslaunch
import std_srvs.srv

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


def shutdown_callback(req):
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


def main():
    global launchRequested
    global launched
    global shutDownRequested

    rospy.init_node('startup_controller')

    launch_service = rospy.Service('controller_launch', std_srvs.srv.Trigger, lauch_callback)
    shutdown_service = rospy.Service('controller_shutdown', std_srvs.srv.Trigger, shutdown_callback)

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
            rospy.loginfo("Shutting down...")
            launch.shutdown()
            launched = False
            shutDownRequested = False
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass