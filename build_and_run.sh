#!/bin/bash

# Function to clean and build the docker image
clean_and_build() {
  # Clean existing images
  docker system prune -y
  docker rmi dynablox

  # Build the docker container
  docker build -t dynablox .
}

# Check if the clean flag is passed
if [ "$1" == "--clean" ]; then
  clean_and_build
fi

# Run the docker container
xhost +local:root
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/spiasecki/Desktop/rosbags:/root/rosbags \
  dynablox