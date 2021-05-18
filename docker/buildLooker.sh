#! /bin/bash

# Check that necessary files and environment is setup
REPOS=misterblue
TAG=ros2-looker
VER=latest
DF=./Dockerfile-looker

# NOCACHE=--no-cache

# PLATFORM=linux/amd64,linux/arm64
PLATFORM=linux/arm64

docker buildx build ${NOCACHE} -t "${REPOS}/${TAG}:${VER}" --push --platform ${PLATFORM} -f "${DF}" .
