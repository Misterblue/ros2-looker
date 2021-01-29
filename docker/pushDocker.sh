#! /bin/bash
# A script to tag local images with the destination repository
#    and push them.
# This script is not needed as buildx is setup to do a --push

REPO=misterblue

echo "Pushing docker image for arm things"

for image in "ros2-base" "ros2-looker-base" "ros2-looker" ; do
    for tagg in latest ; do
        docker tag ${image} ${REPO}/${image}:${tagg}
        echo "   Pushing ${REPO}/${image}:${tagg}"
        docker push ${REPO}/${image}:${tagg}
    done
done
