# GhostNet-ROS2 Wrapper

This is a ROS2 wrapper for the image classification algorithm, [GhostNet](https://arxiv.org/abs/1911.11907). We utilize `pytorch` and `torch.hub` for the [source of the algorithm](https://pytorch.org/hub/pytorch_vision_ghostnet/). The main idea is for this container to act as a standalone interface and node, removing the necessity to integrate separate packages and solve numerous dependency issues.

# Installation Guide

## Using Docker Pull
1. Install [Docker](https://www.docker.com/) and ensure the Docker daemon is running in the background.
2. Run ```docker pull shaderobotics/ghostnet:${ROS2_DISTRO}``` we support all ROS2 distributions
3. Follow the run commands in the usage section below

## Build Docker Image Natively
1. Install [Docker](https://www.docker.com/) and ensure the Docker daemon is running in the background.
2. Clone this repo with ```git pull -b ${ROS2_DISTRO} https://github.com/open-shade/ghostnet.git```
3. Enter the repo with ```cd ghostnet```
4. Build the container with ```docker build . -t [name]```. This will take a while. We have also provided associated `cloudbuild.sh` scripts to build on GCP.
5. Follow the run commands in the usage section below.

# Usage
## Run the GhostNet Node 
Run ```docker run -t ghostnet```. Your node should be running now. Then, by running ```ros2 topic list,``` you should see all the possible pub and sub routes.

For more details explaining how to run Docker images, visit the official Docker documentation [here](https://docs.docker.com/engine/reference/run/). Also, additional information as to how ROS2 communicates between external environment or multiple docker containers, visit the official ROS2 (foxy) docs [here](https://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html#). 

# Topics

| Name                   | IO  | Type                             | Use                                                               |
|------------------------|-----|----------------------------------|-------------------------------------------------------------------|
| ghostnet/image_raw       | sub | [sensor_msgs.msg.Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)            | Takes the raw camera output to be processed                       |
 | ghostnet/result           | pub | String            | Outputs the classification label from ImageNet 100 Classes as a string |

# Testing / Demo
To test and ensure that this package is properly installed, replace the Dockerfile in the root of this repo with what exists in the demo folder. Installed in the demo image contains a [camera stream emulator](https://github.com/klintan/ros2_video_streamer) by [klintan](https://github.com/klintan) which directly pubs images to the GhostNet node and processes it for you to observe the outputs.

To run this, run ```docker build . -t [name]```, then ```docker run -t [name]```. Observing the logs for this will show you what is occuring within the container. If you wish to enter the running container and preform other activities, run ```docker ps```, find the id of the running container, then run ```docker exec -it [containerId] /bin/bash```