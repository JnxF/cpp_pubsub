# This is a sample Dockerfile with a couple of problems.
# Paste your Dockerfile here.

FROM ros:eloquent
ENV DEBIAN_FRONTEND noninteractive


# Install tools
RUN apt-get update > /dev/null \
	&& apt-get install -y --no-install-recommends apt-utils > /dev/null \
	&& apt-get install -y afl > /dev/null \
	&& apt-get install -y vim > /dev/null \
	&& apt-get install -y git > /dev/null \
	&& apt-get install -y clang > /dev/null \
	&& apt-get install -y iproute2 > /dev/null \
	&& apt-get install -y lcov > /dev/null \
	&& apt-get install -y graphviz-dev > /dev/null

# Initialize ROS environment
RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash"

ENV ROS_WS /opt/ros_ws

# Create /opt/ros_ws/src directory
RUN mkdir -p $ROS_WS/src

# And set that to the working directory
WORKDIR $ROS_WS

# Copy this same folder to there
COPY . src/cpp_pubsub

RUN chmod +x src/cpp_pubsub/afl-config.bash
