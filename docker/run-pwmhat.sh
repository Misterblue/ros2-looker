#! /bin/bash
# Start the PWM controller that outputs angle commands to the PWMHAT
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

# The docker container must be able to access the i2c device
sudo chmod 666 /dev/i2c-1

docker run \
    -t \
    -d \
    --rm \
    --name=ros2-adafruit_pwmhat-node \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    -e LOOKER_PACKAGE=ros2_adafruit_pwmhat_node \
    --network=host \
    --privileged \
    misterblue/ros2-looker:latest

#    --restart=unless-stopped
    
