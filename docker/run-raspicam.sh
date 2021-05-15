#! /bin/bash
# Start the raspicam ROS2 node
# The ROS_DOMAIN_ID is set to 30 or the environment variable unless there is
#    a parameter on the invocation of this script:
#         ./run-raspicam.sh 31

# Set the ROS_DOMAIN_ID to first parameter or the environment
if [[ ! -z "$1" ]] ; then
    ROS_DOMAIN_ID=$1
else
    ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
fi
echo "Setting ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

docker run \
    -t \
    -d \
    --rm \
    --name=ros2-raspicam-node \
    -e ROS_DOMAIN=${ROS_DOMAIN_ID} \
    --network=host \
    --privileged \
    misterblue/ros2-looker:latest
    ros2-raspicam-node

#    --restart=unless-stopped
    
