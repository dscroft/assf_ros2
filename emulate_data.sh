#!/bin/bash

zipfile="/tmp/rosbag.zip"
bagfile="/home/$USER/bagfile/"
url="https://livewarwickac-my.sharepoint.com/:u:/g/personal/u2273650_live_warwick_ac_uk/EQJ9C5Gok7VHlfo-OXWzdVYBDvJJWbBXhrgMUhJPy0HXpg?download=1"

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
