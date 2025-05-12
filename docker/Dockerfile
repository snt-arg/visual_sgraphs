FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

##### Environment variables #####
ENV CUDA_HOME=/usr/local/cuda

##### Essential packages #####
RUN apt-get update && apt-get install -y \
    python3-pip \
    python-is-python3 \
    git \
    openssh-client \
    wget \
    vim \
    curl \
    libeigen3-dev \
    build-essential

##### Install ROS 1 - Noetic #####
# setup environment
ENV ROS_DISTRO=noetic

# setup keys
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# ROS related packages 
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-pcl-ros \
    ros-noetic-backward-ros \
    ros-noetic-rviz-visual-tools \
    ros-noetic-hector-trajectory-server

# ROS dependencies
RUN rosdep init
RUN rosdep update

##### Python environment setup #####
# PyTorch and related libraries - networkx needs to be installed first because of version issue
RUN pip3 install networkx==3.1
RUN pip3 install torch==2.0.1+cu118 -f  https://download.pytorch.org/whl/torch_stable.html
RUN pip3 install torchvision==0.15.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118

# detectron and CLIP
# Compute Capability 7.5 for T600 (SnT laptop) and 7.0 for V100 (HPC - Iris)
ARG TORCH_CUDA_ARCH_LIST="7.5;7.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install 'git+https://github.com/facebookresearch/detectron2.git'
RUN pip3 install 'git+https://github.com/openai/CLIP.git'

# other libraries
ADD requirements.txt requirements.txt
RUN pip3 install -r requirements.txt

##### SSH keys for GitHub #####
# ssh keys as build arguments
ARG ssh_prv_key
ARG ssh_pub_key

RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -H github.com >> /root/.ssh/known_hosts

RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa.pub
    
##### Workspace setup #####
RUN mkdir -p /workspace/src

##### Clone repositories #####
# install pangolin
RUN apt-get install libepoxy-dev -y
WORKDIR /workspace/
RUN git clone --branch v0.9 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

# for ROS packages: visual sgraphs, semantic segmenter, aruco ros
WORKDIR /workspace/src/
RUN git clone git@github.com:snt-arg/visual_sgraphs.git
RUN git clone git@github.com:snt-arg/scene_segment_ros.git
RUN git clone -b noetic-devel git@github.com:pal-robotics/aruco_ros.git

# for ROS package: mav_voxblox_planning
RUN git clone git@github.com:snt-arg/mav_voxblox_planning.git
RUN wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall
RUN wstool update

# copy the semantic segmentation models/checkpoints
COPY seg_models/* /workspace/src/scene_segment_ros/include/

# build the workspace
WORKDIR /workspace/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCMAKE_BUILD_TYPE=Release && rosclean purge -y"

##### Miscalleanous #####
RUN ldconfig

##### Clean up #####
# remove the apt list files
RUN rm -rf /var/lib/apt/lists/*

# remove packages no longer needed 
RUN apt-get clean && apt-get autoremove -y

# remove the ssh keys
RUN rm -rf /root/.ssh/

##### Build entrypoint #####
RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "source /opt/ros/noetic/setup.bash" >> /entrypoint.sh \
    && echo "source /workspace/devel/setup.bash" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

WORKDIR /workspace/

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]