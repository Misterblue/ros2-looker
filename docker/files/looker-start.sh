#! /bin/bash

source /opt/ros/foxy/setup.bash
source ros2-looker/install/setup.bash

PACKAGE=$1
EXE=${2:-service}

for pkg in "ros2_adafruit_pwmhat_node" "ros2_facelook_node" "ros2_facefinder_node" "ros2_raspicam_node" ; do
    if [[ "$PACKAGE" == "$pkg" ]] ; then
        OK=yes
    fi
done

if [[ "$OK" != "yes" ]] ; then
    echo "Specified package is not known. Package=$PACKAGE"
    exit 5
fi

echo "Running executable $EXE in package $PACKAGE"
ros2 run "$PACKAGE" "$EXE"
