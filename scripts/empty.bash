#!/bin/bash -e

if [ -s /opt/ros/noetic/setup.bash ]; then
        # The file is not-empty.
        echo "/opt/ros/noetic/setup.bash exists!"
else
        # The file is empty.
        mkdir -p /opt/ros/noetic/
        touch /opt/ros/noetic/setup.bash
fi
