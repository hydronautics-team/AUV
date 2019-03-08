#!/bin/bash

# TODO: Add "install" command
# TODO: Check argument repeating

source devel/setup.bash

function show_help()
{
    printf "\n\nSimple AUV managment script. Usage:\n"

    printf "./auv-admin.sh run <mode> [--sim] [--stream] [--nocon] [--nocam]\n"
    printf "Runs nodes. Parameters and arguments:\n"
    printf "    mode - Application mode. Available modes: qualification_simple, qualification_vision, missions, demo, none.\n"
    printf "    --sim - Optional argument, enables simulation mode.\n"
    printf "    --stream - Optional argument, enables video stream from cameras.\n"
    printf "    --nocon - Optional argument, disables connection with hardware interfaces.\n"
    printf "    --nocam - Optional argument, disables cameras.\n"
    printf "\n"
}

STATE_BEGIN=0
STATE_RUN_MODE=1 # parsing launch mode
STATE_OPT_ARG=3 # parsing optional arg

CURRENT_STATE=$STATE_BEGIN
NEXT_STATE=$STATE_BEGIN

LAUNCH_COMMAND=" "

while [ "$1" != "" ]; do

    case $CURRENT_STATE in

        $STATE_BEGIN )

            case "$1" in
                "run" )
                    NEXT_STATE=${STATE_RUN_MODE}
                    ;;

                "help" )
                    show_help
                    exit
                    ;;

                * )
                    echo "Unknown command. See help."
                    exit
                    ;;
            esac

            ;;

        $STATE_RUN_MODE )

            case "$1" in
                "qualification_simple" | "qualifs" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND mode:=qualification_simple"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                "qualification_vision" | "qualifv" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND mode:=qualification_simple"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                "missions" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND mode:=missions"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                "demo" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND mode:=demo"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                "none" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND mode:=none"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                * )
                    echo "Unknown mode. See help."
                    exit
                    ;;
             esac

            ;;

        $STATE_OPT_ARG )

             case "$1" in
                "--simulation" | "--sim" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND simulation:=true"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                "--stream" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND stream:=true"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                 "--nocon" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND connectionDisabled:=true"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                 "--nocam" )
                    LAUNCH_COMMAND="$LAUNCH_COMMAND camerasDisabled:=true"
                    NEXT_STATE=${STATE_OPT_ARG}
                    ;;
                 * )
                    echo "Unknown argument $1. See help."
                    exit
                    ;;
             esac

            ;;

    esac
    CURRENT_STATE=$NEXT_STATE
    shift
done

if [[ $CURRENT_STATE == $STATE_RUN_MODE ]]
then
    echo "Missing mode. See help."
    exit
fi

LAUNCH_COMMAND="$LAUNCH_COMMAND imuReset:=true"
echo $LAUNCH_COMMAND
roslaunch launch/AUV.launch $LAUNCH_COMMAND