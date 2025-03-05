#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

zipfile="/tmp/rosbag.zip"
bagfile="$DIR/.bagfile/"
url="https://livewarwickac-my.sharepoint.com/:u:/g/personal/u2273650_live_warwick_ac_uk/EUsLPO6MLKNJpe56JZ-norgBkYKj3hZZ-wn8w-K2e-0elA?download=1"

# if bagfile directory does not exist, create it
if [ ! -d "$bagfile" ]; then
    # if zipfile does not exist, get it
    if [ ! -f "$zipfile" ]; then
        echo "Downloading data file"
        wget $url -O $zipfile

        if [ $? -ne 0 ]; then
            echo "Failed to download data file"
            rm $zipfile 2> /dev/null
            exit 1
        fi
    fi

    echo "Extracting data file"
    unzip $zipfile -d $bagfile
fi

ros2 bag play --loop $( ls -1 $bagfile | head -n 1 | sed "s|^|$bagfile|")
