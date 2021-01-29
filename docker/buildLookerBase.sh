#! /bin/bash

# Check that necessary files and environment is setup
REPOS=misterblue
TAG=ros2-looker-base
VER=latest
DF=./Dockerfile-looker-base

# NOCACHE=--no-cache

docker buildx build ${NOCACHE} -t "${REPOS}/${TAG}:${VER}" --push --platform linux/amd64,linux/arm64 -f "${DF}" .
# docker buildx build --load -t "${TAG}:${VER}" -f "${DF}" .
