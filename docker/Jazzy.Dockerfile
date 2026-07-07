FROM nvcr.io/nvidia/cuda-dl-base:25.04-cuda12.9-devel-ubuntu24.04

# Arguments
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

# Environment variables
ENV CUDA_HOME=/usr/local/cuda \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=jazzy \
    PIP_BREAK_SYSTEM_PACKAGES=1

# --- Fix MPI issue ---
RUN mkdir -p /opt/hpcx/ompi/lib/x86_64-linux-gnu \
    && ln -s /opt/hpcx/ompi /opt/hpcx/ompi/lib/x86_64-linux-gnu/openmpi \
    && dpkg-reconfigure libc-bin

# --- Handle user creation ---
RUN if id -u $USER_UID ; then userdel "$(id -un $USER_UID)" ; fi

# --- System setup ---
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
    locales \
    software-properties-common \
    lsb-release \
    gnupg2 && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN add-apt-repository universe

# --- ROS 2 Jazzy APT source setup ---
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt install -y /tmp/ros2-apt-source.deb

# --- Install ROS 2 Jazzy development tools ---
RUN apt update && apt upgrade -y && \
    apt install -y ros-dev-tools ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-rqt-tf-tree

# --- Source ROS globally ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# --- Initialize rosdep ---
RUN rosdep init && rosdep update

# --- Clean up ---
RUN rm -rf /var/lib/apt/lists/* /tmp/*

# --- Create user ---
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# --- Python environment setup ---
RUN pip3 install networkx==3.1
RUN pip3 install --index-url https://download.pytorch.org/whl/cu121 \
    torch==2.5.1 \
    torchvision==0.20.1
RUN apt remove --purge python3-typing-extensions -y
RUN pip3 install typing-extensions==4.11.0

# --- CLIP and Detectron2 setup ---
ARG TORCH_CUDA_ARCH_LIST="7.5;7.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install 'git+https://github.com/facebookresearch/detectron2.git@fd27788985af0f4ca800bca563acdb700bb890e2'
RUN pip3 install 'git+https://github.com/openai/CLIP.git'


# --- SSH keys ---
# Define the SSH keys as build arguments for latter mounting
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# --- Clone repositories ---
RUN apt-get update && apt-get install -y \
    libepoxy-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglew-dev \
    cmake \
    build-essential \
    git

# Pangolin
WORKDIR /opt/
RUN git clone --branch v0.9.1 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

# Cmake
ARG version=3.22
ARG build=1
WORKDIR /tmp
RUN wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz

RUN tar -xzvf cmake-$version.$build.tar.gz
WORKDIR /tmp/cmake-$version.$build
RUN ./bootstrap
RUN make -j8
RUN make install

WORKDIR /home/$USERNAME/workspace/src

# Mount the SSH keys and clone the vS-Graphs repositories
RUN --mount=type=ssh git clone git@github.com:snt-arg/visual_sgraphs.git
RUN --mount=type=ssh git clone git@github.com:snt-arg/situational_graphs_msgs.git
RUN --mount=type=ssh git clone git@github.com:snt-arg/scene_segment_ros.git
RUN --mount=type=ssh git clone -b r/4.57.7 git@github.com:IntelRealSense/realsense-ros.git

# Install the vS-Graphs dependencies
WORKDIR /home/$USERNAME/workspace/src/visual_sgraphs/docker
RUN pip3 install --break-system-packages -r requirements.txt --upgrade-strategy only-if-needed

WORKDIR /home/$USERNAME/workspace/src/

# Download the yoso checkpoint
RUN wget https://github.com/hujiecpp/YOSO/releases/download/v0.1/yoso_res50_coco.pth
RUN mv yoso_res50_coco.pth /home/$USERNAME/workspace/src/scene_segment_ros/include/

# USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-pcl-ros

# Build the workspace
WORKDIR /home/$USERNAME/workspace/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# --- Miscalleanous ---
RUN ldconfig

# --- Clean up ---
# Remove the apt list files
RUN rm -rf /var/lib/apt/lists/*

# Remove packages no longer needed
RUN apt-get clean && apt-get autoremove -y

# Remove the ssh keys
RUN rm -rf /root/.ssh/

# --- Build entrypoint ---
RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "echo \"source /opt/ros/$ROS_DISTRO/setup.bash\" >> ~/.bashrc" >> /entrypoint.sh \
    && echo "echo \"source /home/$USERNAME/workspace/install/setup.bash\" >> ~/.bashrc" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

USER $USERNAME
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace/

# ---------------------------
# Download and Install mprocs
# ---------------------------
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y \
    && . "$HOME/.cargo/env" \
    && cargo install mprocs

# --------------------------
# Aliases and Environment Setup
# --------------------------
RUN echo "alias mprocs='mprocs -c /home/$USERNAME/workspace/src/visual_sgraphs/config/mprocs.yml'" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
USER $USERNAME
CMD ["/bin/bash"]
SHELL ["/bin/bash"]