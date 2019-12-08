#!/bin/bash

# Check if two arguments were passed in
if [[ $# -ne 2 ]]; then
  echo "Please pass in arguments for the workspace and package names"
  exit 1
fi
# Assign the passed in arguments to variables
ws=${1:-test_ws}
pkg=${2:-test_pkg}

# Check if there is a package source directory to link to
read -p "Do you have a separate package source directory ~/work/${pkg}/src to link to? (y/n) " answer
if [[ ${answer} != "y" ]]; then
  echo "Please create the package source directory and rerun"
  exit 1
fi

# Create the workspace
cd ${HOME}/work
mkdir ${ws}
cd ${ws}
mkdir src
catkin_make
source devel/setup.bash

# Create the package
cd src/
catkin_create_pkg ${pkg} st_msgs rospy roscpp
cd ${pkg}
rm -rf src
ln -s ${HOME}/work/${pkg}/src .
echo "Run this: source ${HOME}/work/${ws}/devel/setup.bash"
