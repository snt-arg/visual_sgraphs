FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

# ----- Environment Variables -----
ENV CUDA_HOME=/usr/local/cuda
ENV ROS_DISTRO=noetic
ENV FORCE_CUDA="1"
ARG TORCH_CUDA_ARCH_LIST="7.5;7.0+PTX"

# ----- Essential Packages -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python-is-python3 \
    git \
    openssh-client \
    wget \
    vim \
    curl \
    libeigen3-dev \
    build-essential \
    libepoxy-dev && \
    rm -rf /var/lib/apt/lists/*

# ----- Install ROS Noetic -----
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-pcl-ros \
    ros-noetic-backward-ros \
    ros-noetic-rviz-visual-tools \
    ros-noetic-hector-trajectory-server && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# ----- Python Packages -----
COPY requirements.txt requirements.txt
RUN pip3 install --no-cache-dir networkx==3.1 && \
    pip3 install --no-cache-dir torch==2.0.1+cu118 -f https://download.pytorch.org/whl/torch_stable.html && \
    pip3 install --no-cache-dir torchvision==0.15.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118 && \
    pip3 install --no-cache-dir \
    'git+https://github.com/facebookresearch/detectron2.git' \
    'git+https://github.com/openai/CLIP.git' && \
    pip3 install --no-cache-dir -r requirements.txt

# ----- SSH Keys for Private Git Repos -----
ARG ssh_prv_key
ARG ssh_pub_key
RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -H github.com >> /root/.ssh/known_hosts && \
    echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa /root/.ssh/id_rsa.pub

# ----- Workspace Setup -----
RUN mkdir -p /workspace/src
WORKDIR /workspace

# ----- Clone and Build External Libraries -----
RUN git clone --branch v0.9 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && \
    cmake .. && make -j && make install

# ----- Clone ROS Packages -----
WORKDIR /workspace/src
RUN git clone git@github.com:snt-arg/visual_sgraphs.git && \
    git clone git@github.com:snt-arg/scene_segment_ros.git && \
    git clone -b noetic-devel git@github.com:pal-robotics/aruco_ros.git && \
    git clone git@github.com:snt-arg/mav_voxblox_planning.git

RUN wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall && \
    wstool update

# ----- Copy Segmentation Models -----
COPY seg_models/* /workspace/src/scene_segment_ros/include/

# ----- Build the Catkin Workspace -----
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCMAKE_BUILD_TYPE=Release && rosclean purge -y"

# ----- Miscellaneous -----
RUN ldconfig

# ----- Clean Up -----
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    apt-get autoremove -y && \
    rm -rf /root/.ssh/

# ----- Entrypoint Setup -----
RUN echo "#!/bin/bash" >> /entrypoint.sh && \
    echo "source /opt/ros/noetic/setup.bash" >> /entrypoint.sh && \
    echo "source /workspace/devel/setup.bash" >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod a+x /entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]