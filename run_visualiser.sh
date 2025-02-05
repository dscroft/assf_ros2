#!/bin/bash

ws="/home/$USER/visualise_ws"

# create workspace
mkdir -p $ws/src
cd $ws/src

# clone packages
if [ ! -d "rosboard" ]; then
    git clone https://github.com/dscroft/rosboard.git
fi

# build in release mode
cd $ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# source workspace
source $ws/install/setup.bash

ros2 run rosboard rosboard_node

