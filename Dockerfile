# This is a sample Dockerfile with a couple of problems.
# Paste your Dockerfile here.

FROM ros:eloquent

RUN apt-get update > /dev/null && apt-get install -y vim > /dev/null

RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash"

ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

COPY . src/cpp_pubsub
