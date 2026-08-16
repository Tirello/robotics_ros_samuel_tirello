#!/bin/bash
# Use Bash as the shell for this script.

# --network=host \      -> Use the host's network, not creating a separate network stack for the Docker container.
# -f $DOCKERFILE \      -> Specify the Dockerfile to use.
# -t $IMAGE_NAME:$IMAGE_TAG \ -> Tag the built image.
# --rm \                -> Remove intermediate containers after a successful build.
# Build a Docker image from a Dockerfile located in the current directory.

CONFIGFILE=docker/config/tools.sh
source $CONFIGFILE

docker build \
    --network=host \
    -f $DOCKERFILE \
    -t $IMAGE_NAME:$IMAGE_TAG \
    --rm \
    .
