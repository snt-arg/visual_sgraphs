FROM amd64/ros:noetic-perception-focal AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common git \
        build-essential cmake libeigen3-dev \
        python3-catkin-tools libopencv-dev \
        libssl-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        librealsense2-dev \
        librealsense2-utils \
        ros-${ROS_DISTRO}-realsense2-camera  \
        ros-${ROS_DISTRO}-hector-trajectory-server \

WORKDIR /deps
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

WORKDIR /ws

RUN  git clone https://github.com/thien94/orb_slam3_ros.git && \
    cd .. && catkin config --extend /opt/ros/noetic && \
    catkin build

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /ws/devel/setup.bash" >> ~/.bashrc


# Build Entrypoint
RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "source /opt/ros/noetic/setup.bash" >> /entrypoint.sh \
    && echo "source /ws/devel/setup.bash" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

WORKDIR /ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
