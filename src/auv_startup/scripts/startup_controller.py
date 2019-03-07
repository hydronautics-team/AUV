#! /usr/bin/env python

import rospy
import roslaunch
import os
import std_srvs.srv
import auv_startup.srv

launchRequested = False
launched = False
shutDownRequested = False

launchFilePath = '/home/sibirsky/factory/AUV/launch/AUV.launch'
launchArgs = []

def lauch_callback(req):
    global launchRequested
    global launched
    global launchArgs
    resp = std_srvs.srv.TriggerResponse

    if launchRequested:
        resp.success = False
        resp.message = "Launch is already requested"
        return resp

    if launched:
        resp.success = False
        resp.message = "Already launched"
        return resp

    mode = req.mode.lower()
    if mode not in ['qualification_simple', 'qualification_vision', 'missions', 'none']:
        resp.success = False
        resp.message = 'Unknown mode. Available modes: qualification_simple, qualification_vision, missions, none'
        return resp

    #launchArgs = [launchFilePath, 'mode:=' + mode, 'imuReset:=true']
    launchRequested = True
    resp.success = True


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

    launch_service = rospy.Service('controller_launch', auv_startup.srv.LaunchCmd, lauch_callback)
    stop_service = rospy.Service('controller_stop', std_srvs.srv.Trigger, stop_callback)
    poweroff_service = rospy.Service('controller_poweroff', std_srvs.srv.Trigger, poweroff_callback)

    parent = None

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if launchRequested and not launched:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            #launchArgs = ['/home/sibirsky/factory/AUV/launch/AUV.launch', 'mode:=none']
            #roslaunchArgs = launchArgs[1:]
            #roslaunchFile = [(roslaunch.rlutil.resolve_launch_arguments(launchArgs)[0], roslaunchArgs)]
            cli_args = ['/home/sibirsky/factory/AUV/launch/AUV.launch', 'mode:=none']
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            rospy.loginfo("Launching...")
            parent.start()
            launched = True
            launchRequested = False
        elif shutDownRequested and launched:
            rospy.loginfo("Stopping launch config...")
            parent.shutdown()
            launched = False
            shutDownRequested = False
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass