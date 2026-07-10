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

ENV PATH="/usr/src/tensorrt/bin:${PATH}"

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
RUN pip3 uninstall -y torch torchvision torchaudio || true
RUN pip3 install --extra-index-url https://download.pytorch.org/whl/cu126 \
    torch==2.6.0+cu126 \
    torchvision==0.21.0+cu126
RUN apt remove --purge python3-typing-extensions -y
RUN pip3 install typing-extensions==4.11.0

# --- CLIP and Detectron2 setup ---
ARG TORCH_CUDA_ARCH_LIST="7.5;7.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install 'git+https://github.com/facebookresearch/detectron2.git'
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
RUN --mount=type=ssh git clone -b ros2-jazzy git@github.com:snt-arg/scene_segment_ros.git
RUN --mount=type=ssh git clone -b r/4.57.2 --depth 1 git@github.com:IntelRealSense/realsense-ros.git
# RUN --mount=type=ssh git clone -b humble-devel git@github.com:pal-robotics/aruco_ros.git

# Repositories for GNN-based room detection and reasoning
# RUN --mount=type=ssh git clone -b develop git@github.com:snt-arg/situational_graphs_wrapper.git
# RUN --mount=type=ssh git clone -b develop git@github.com:snt-arg/situational_graphs_datasets.git
# RUN --mount=type=ssh git clone -b develop git@github.com:snt-arg/situational_graphs_reasoning.git
# RUN --mount=type=ssh git clone -b main git@github.com:snt-arg/situational_graphs_reasoning_msgs.git

# Install the vS-Graphs dependencies
WORKDIR /home/$USERNAME/workspace/src/visual_sgraphs/docker
RUN pip3 install --break-system-packages --ignore-installed -r requirements.txt

# Install object tracker dependencies when the package is present in the workspace
RUN if [ -f /home/$USERNAME/workspace/src/object_tracker_3d_ros/requirements.txt ]; then \
      pip3 install --break-system-packages --ignore-installed \
      -r /home/$USERNAME/workspace/src/object_tracker_3d_ros/requirements.txt ; \
    fi

# [Hint] Temp. fix for installing ROS2 Humble repositories (GNN-based room detection) in Jazzy
# (Read more: https://github.com/ros2/ros2/issues/1702)
# RUN pip3 install --break-system-packages setuptools==79.0.1

# Install reasoning dependencies
# RUN pip3 install --break-system-packages shapely==2.1.1 torch-geometric==2.6.1 transforms3d==0.4.2
# RUN mkdir -p /home/$USERNAME/workspace/install/situational_graphs_reasoning/share/situational_graphs_reasoning/reports \
#     && chown -R $USERNAME:$USERNAME /home/$USERNAME/workspace/install/situational_graphs_reasoning/share/situational_graphs_reasoning/reports

# Install the EOMT dependencies
WORKDIR /home/$USERNAME/workspace/src/scene_segment_ros/src/
RUN git clone https://github.com/tue-mps/eomt.git
RUN pip3 install --ignore-installed -r /home/${USERNAME}/workspace/src/scene_segment_ros/src/eomt/requirements.txt
RUN pip3 install "numpy<2.0" --force-reinstall

# WORKDIR /home/$USERNAME/workspace/src/

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
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip realsense2_ros_mqtt_bridge"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select realsense2_ros_mqtt_bridge"

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

# ------------------------------------
# Download Vox2Ros Toolkit for Voxblox
# ------------------------------------
WORKDIR /home/$USERNAME/workspace/vsgraphs_tools
RUN curl -L https://raw.githubusercontent.com/snt-arg/vsgraphs_tools/refs/heads/main/Voxblox/relay_jazzy.py -o /home/$USERNAME/workspace/vsgraphs_tools/relay_jazzy.py
RUN chmod +x /home/$USERNAME/workspace/vsgraphs_tools/relay_jazzy.py

USER $USERNAME
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace/

# ---------------------------
# Download and Install mprocs
# ---------------------------
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y \
    && . "$HOME/.cargo/env" \
    && cargo install mprocs


# note: you might have to reinstall numpy and opencv python pkgs and rebuilding cause the system packages are pointed instead
# note: torch version might be wrong (cu130 wheel), reinstall torch and torchvision with the correct versions if you encounter issues related to torch.
RUN sudo rm -rf \
  /usr/local/lib/python3.12/dist-packages/torch \
  /usr/local/lib/python3.12/dist-packages/torch-* \
  /usr/local/lib/python3.12/dist-packages/torchvision \
  /usr/local/lib/python3.12/dist-packages/torchvision-* \
  /usr/local/lib/python3.12/dist-packages/torchaudio \
  /usr/local/lib/python3.12/dist-packages/torchaudio-* \
  /usr/local/lib/python3.12/dist-packages/functorch \
  /usr/local/lib/python3.12/dist-packages/torchgen

RUN python3 -m pip install --user --break-system-packages --no-cache-dir --force-reinstall \
  torch torchvision torchaudio \
  --index-url https://download.pytorch.org/whl/cu126

# Purge numpy/scipy from BOTH the user site and the system dist-packages before
# reinstalling. Earlier layers install numpy/scipy system-wide with
# `--ignore-installed`, which does not clean up files from a differently-laid-out
# previous version. If those stale files are left in dist-packages, they can
# shadow the pinned --user install below (e.g. a stray flat
# scipy/sparse/linalg/_propack*.so shadowing the real _propack/ package),
# breaking `import scipy` at runtime even though `pip list` looks correct.
RUN rm -rf ~/.local/lib/python3.12/site-packages/numpy \
       ~/.local/lib/python3.12/site-packages/numpy-*.dist-info \
       ~/.local/lib/python3.12/site-packages/numpy.libs \
       ~/.local/lib/python3.12/site-packages/scipy \
       ~/.local/lib/python3.12/site-packages/scipy-*.dist-info \
       ~/.local/lib/python3.12/site-packages/scipy.libs
RUN sudo rm -rf \
  /usr/local/lib/python3.12/dist-packages/numpy \
  /usr/local/lib/python3.12/dist-packages/numpy-* \
  /usr/local/lib/python3.12/dist-packages/numpy.libs \
  /usr/local/lib/python3.12/dist-packages/scipy \
  /usr/local/lib/python3.12/dist-packages/scipy-* \
  /usr/local/lib/python3.12/dist-packages/scipy.libs

RUN python3 -m pip install --user --break-system-packages --no-cache-dir "numpy==1.26.4"
RUN python3 -m pip install --user --break-system-packages --no-cache-dir --force-reinstall --no-deps \
  "scipy==1.13.1"
RUN python3 -m pip install --user --break-system-packages --no-cache-dir --force-reinstall --no-deps \
  "ultralytics==8.4.68"

WORKDIR /home/$USERNAME/workspace/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip realsense2_ros_mqtt_bridge"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select realsense2_ros_mqtt_bridge"

# --------------------------
# Aliases and Environment Setup
# --------------------------
RUN echo "alias mprocs='mprocs -c /home/$USERNAME/workspace/src/visual_sgraphs/config/mprocs.yml'" >> ~/.bashrc && \
    echo "alias rel_vox='python /home/$USERNAME/workspace/vsgraphs_tools/relay_jazzy.py --mode voxblox_client'" >> ~/.bashrc && \
    echo "alias rel_pcl='python /home/$USERNAME/workspace/vsgraphs_tools/relay_jazzy.py --mode pc_server'" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
USER $USERNAME
CMD ["/bin/bash"]
SHELL ["/bin/bash"]
