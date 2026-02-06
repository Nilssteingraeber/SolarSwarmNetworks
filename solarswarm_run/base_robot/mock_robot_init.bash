#!/bin/bash
# source /opt/ros/jazzy/setup.bash
# python3 unzip.py "install.zip"

# set envs
ENVS=/var/tmp/solarswarm/env

sed -i '/#/d; /;/d; /&/d; /|/d' $ENVS # remove lines containing '#', ';', '&', or '|'

lines=$(wc -l < $ENVS) # solution to only get number https://stackoverflow.com/a/15882198
if [ $? ]; then
    echo $lines
    for i in $(seq $lines); do
        line=$(head -$i $ENVS | tail -$i)
        if echo $line | grep -E "[A-Za-z0-9_]+=(\d+|[A-Za-z0-9_./-]+|\"[^\"]*\")" &>/dev/null; then
            # left of '=' should permit a name for an env
            # right of '=' should permit a number, path, or string
            # []+ for at least one of: alphanum character or '_'
            # (|)+ to group two options and repeat
            # \d for a number
            # only allow values like WLANDEV=abc, MESH_IP=192.168.1.100, etc.

            # is valid (probably)
            sudo echo $line >> /etc/environment
        fi
    done
    source install/setup.bash # source ROS packages
    ros2 run mock_robot mock_data # run ROS package
else
    echo "COuld not find $ENVS"
    exit 1
fi