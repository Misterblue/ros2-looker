#! /bin/bash
# Start the face looker who converts the found faces into PWM angle commands
# The ROS_DOMAIN_ID is set to 30 or the environment variable unless there is
#    a parameter on the invocation of this script:
#         ./run-facefinder.sh 31

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
    --name=ros2-facelook-node \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    -e LOOKER_PACKAGE=ros2_facelook_node \
    -e LOOKER_DEBUG=${LOOKER_DEBUG:-no} \
    --network=host \
    --privileged \
    misterblue/ros2-looker:latest

#    --restart=unless-stopped
    
