#!/bin/bash

# get dir of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

for d in $DIR/*_ws; do
    echo "Rebuilding $d"
    cd $d
    echo "Delete build, install, log"
    rm -rf build install log
    echo "Build"
    colcon build
done
