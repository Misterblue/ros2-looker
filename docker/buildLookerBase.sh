#! /bin/bash

# Check that necessary files and environment is setup
REPOS=misterblue
TAG=ros2-looker-base
VER=latest
DF=./Dockerfile-looker-base

NOCACHE=
if [[ "$1" == "NOCACHE" ]] ; then
    NOCACHE=--no-cache
fi 

# PLATFORM=linux/amd64,linux/arm64
PLATFORM=linux/arm64

docker buildx build ${NOCACHE} -t "${REPOS}/${TAG}:${VER}" --push --platform ${PLATFORM} -f "${DF}" .
# docker buildx build --load -t "${TAG}:${VER}" -f "${DF}" .
