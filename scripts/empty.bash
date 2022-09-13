#!/bin/bash -e

if [ -s diff.txt ]; then
        # The file is not-empty.
        echo "/opt/ros/noetic/setup.bash exists!"
else
        # The file is empty.
        touch /opt/ros/noetic/setup.bash
fi
