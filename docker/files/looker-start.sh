#! /bin/bash

source /opt/ros/foxy/setup.bash
source ros2-looker/install/local_setup.bash

PACKAGE=${LOOKER_PACKAGE}
NODE=${LOOKER_NODE:-service}

for pkg in "ros2_adafruit_pwmhat_node" "ros2_facelook_node" "ros2_facefinder_node" "ros2_raspicam_node" ; do
    if [[ "$PACKAGE" == "$pkg" ]] ; then
        OK=yes
    fi
done

if [[ "$OK" != "yes" ]] ; then
    echo "Specified node is not known. Node=$NODE"
    exit 5
fi

echo "Running executable $NODE in package $PACKAGE"
ros2 run "$PACKAGE" "$NODE"

