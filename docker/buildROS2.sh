#! /bin/bash

# Check that necessary files and environment is setup
REPOS=misterblue
TAG=ros2-base
VER=latest
DF=Dockerfile-ros2-base

NOCACHE=
if [[ "$1" == "NOCACHE" ]] ; then
    NOCACHE=--no-cache
fi 


docker buildx build ${NOCACHE} -t "${REPOS}/${TAG}:${VER}" --push --platform linux/amd64,linux/arm64 -f "${DF}" .
# docker buildx build --load -t "${TAG}:${VER}" -f "${DF}" .
