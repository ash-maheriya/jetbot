#!/bin/bash
# Create a ROS workspace 
WSNAME=${1:-${HOME}/work/jetbot_ws}
echo ${WSNAME}
if [ ! -d ${WSNAME}/src ]; then
  mkdir -p ${WSNAME}/src
fi
cd ${WSNAME}
catkin_make
