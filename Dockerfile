# This is a sample Dockerfile with a couple of problems.
# Paste your Dockerfile here.

FROM ros:eloquent

# Install tools
RUN apt-get update > /dev/null \
	&& apt-get install -y vim > /dev/null \
	&& apt-get install -y git > /dev/null

# Initialize ROS environment
RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash"

# Install AFL
RUN mkdir /afl
RUN cd /afl && git clone https://github.com/google/AFL.git
RUN cd /afl/AFL && make
RUN cd /afl/AFL && make install

# Configure AFL
# Slightly based on https://github.com/AlexandreCarlton/afl-docker/blob/master/Dockerfile
ENV CC=afl-gcc
ENV CXX=afl-g++

# Set the ROS working space
ENV ROS_WS /opt/ros_ws

# Create /opt/ros_ws/src directory
RUN mkdir -p $ROS_WS/src

# And set that to the working directory
WORKDIR $ROS_WS

# Copy this same folder to there
COPY . src/cpp_pubsub
