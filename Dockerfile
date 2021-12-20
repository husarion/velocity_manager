FROM ros:noetic-ros-core

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update \
    && apt upgrade -y \
    && apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    libgmock-dev

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg

WORKDIR /app

# Create and initialise ROS workspace
RUN mkdir -p ros_ws/src
RUN git clone https://github.com/husarion/husarion_msgs.git --branch main ros_ws/src/husarion_msgs \
    && cd ros_ws \
    && mkdir build \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make install -DCMAKE_BUILD_TYPE=Release

COPY ./velocity_manager ros_ws/src/velocity_manager

RUN cd ros_ws \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source devel/setup.bash \
    && catkin_make install --pkg velocity_manager -DCMAKE_BUILD_TYPE=Release \
    && rosrun velocity_manager VelocityManagerTest

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /
