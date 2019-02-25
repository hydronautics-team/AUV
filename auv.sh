#!/bin/bash

source devel/setup.bash
if [ "$1" == "--bg" ] 
then
    nohup roslaunch launch/AUV.launch &
else
    roslaunch launch/AUV.launch
fi