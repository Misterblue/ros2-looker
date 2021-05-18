#! /bin/bash

source /opt/ros/foxy/setup.bash
source ros2-looker/install/local_setup.bash

PACKAGE=${LOOKER_PACKAGE}
NODE=${LOOKER_NODE:-service}

# If environment var "LOOKER_DEBUG" is set, output debug messages
IFDEBUG=${LOOKER_DEBUG:-no}

for pkg in "ros2_adafruit_pwmhat_node" "ros2_facelook_node" "ros2_facefinder_node" "ros2_raspicam_node" ; do
    if [[ "$PACKAGE" == "$pkg" ]] ; then
        OK=yes
    fi
done

if [[ "$OK" != "yes" ]] ; then
    echo "Specified node is not known. Node=$NODE"
    exit 5
fi

DEBUGG=""
if [[ "$IFDEBUG" != "no" ]] ; then
    DEBUGG="--ros-args --log-level debug"
fi

echo "Running executable $NODE in package $PACKAGE"
ros2 run "$PACKAGE" "$NODE" ${DEBUGG}

