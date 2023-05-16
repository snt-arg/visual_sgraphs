# Semantic ORB-SLAM3-ROS

A modified version of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) ROS implementation version introduced [here](https://github.com/thien94/orb_slam3_ros) that supports adding semantic entities to the final map and hierarchical presentation.

## 📝 Prerequisites

Install the required libraries that ORB-SLAM 3.0 uses:

### OpenCV

Check the OpenCV version on your computer (required [at least 3.0](https://github.com/UZ-SLAMLab/ORB_SLAM3)):

```
python3 -c "import cv2; print(cv2.__version__)" 
```
On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is already included. If a newer version is required (>= 3.0), follow [installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) and change the corresponding OpenCV version in `CMakeLists.txt`

### Eigen3

Install `Eigen`, which is a C++ template library for linear algebra (including matrices, vectors, and numerical solvers):

```
sudo apt install libeigen3-dev
```

### Pangolin

Install `Pangolin`, which is a set of lightweight and portable utility libraries for prototyping 3D, numeric or video based programs and algorithms:

```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```

### `hector-trajectory-server` (optional)

Using this library you can visualize the real-time trajectory of `camera/IMU`.

```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

### `aruco_ros` (contribution)

This package (available [here](https://github.com/pal-robotics/aruco_ros)) enables you to detect ArUco Markers in cameras' field of view. Accordingly, install it using `sudo apt-get install ros-[DISTRO]-aruco-ros
` or the commands below:

```
# Catkin workspace folder
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/

# Cloning the latest code
git clone git@github.com:pal-robotics/aruco_ros.git
cd aruco_ros/

# Installing the library
catkin init
catkin build

# Sourcing the new configurations
# nano ~/.bashrc -> Add "alias sourcearuco='source ~/workspace/ros/aruco_ros_ws/devel/setup.bash'"
sourcearuco
```

As a quick test, run `roslaunch aruco_ros marker_publisher.launch` for detecting multiple markers in the scene and publishing their poses. You can use the sample modified `marker_publisher.launch` file for this library available [here](doc/aruco_ros_marker_publisher.launch), which works fine with the *UniLu* dataset and the live feed for RealSense cameras (`imageRaw` and `cameraInfo` should be changed). The final results (scene with detected markers) are published and accessible on `/aruco_marker_publisher/result` and the pose of the markers will be shown using `rostopic echo /aruco_marker_publisher/markers`.

[Hint:] Set proper `ref_frame`, `markerSize`, `imageRaw`, and `cameraInfo` values in the launch file.

## ⚙️ Installation

After installing the prerequisites, you can install the repository using commands below:

```
cd ~/[workspace]/src
git clone git@github.com:snt-arg/semantic_orb_slam3_ros.git
cd ../
catkin build
```

Finally, you can add a new alias to the `bashrc` file to run the environment whenever needed:

```
alias sourceorb3ros='source ~/workspace/ros/orbslam3_ros_ws/devel/setup.bash'
```

## 🔨 Configurations

You can find the configuration files for the application in the `config` folder. It contains some `Calibration` processes, camera parameter for various sensors, and some `rviz` files for different datasets. You can define your own `rviz` and `yaml` files according to the characteristics of the sensor you collected data with. A sample of how to set camera intrinsic and extrinsic parameters can be found [here](https://github.com/shanpenghui/ORB_SLAM3_Fixed#73-set-camera-intrinsic--extrinsic-parameters).

## 🚀 Run Examples

| Mode | Dataset | Commands | Notes |
| ------------ | ------------ | ------------ | ------------ |
| Mono | [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)'s [`eee_01.bag`](https://researchdata.ntu.edu.sg/api/access/datafile/68133) | `roslaunch orb_slam3_ros ntuviral_mono.launch` <br /> `rosbag play eee_01.bag -s 50` | - |
| Mono | [UniLu](https://uniluxembourg-my.sharepoint.com/:f:/g/personal/ali_tourani_uni_lu/EpmLAimeFdxAgXk4TAhSxEMBi4b1JOoWrWW_KFWewmyuNQ?e=l2Larj)'s Test Offices | `roslaunch orb_slam3_ros unilu_mono.launch` <br /> `rosbag play Test-Offices.bag --clock` | data collected by RealSense from ARG offices |
| Mono | Live with ArUco markers | `roslaunch orb_slam3_ros live_rs_d435i.launch` <br /> `roslaunch aruco_ros marker_publisher.launch` <br /> `roslaunch realsense2_camera rs_rgbd.launch align_depth:=true unite_imu_method:=linear_interpolation` | change `imageRaw` and `cameraInfo` to `/camera/color/image_raw` and `/camera/aligned_depth_to_color/camera_info` in `marker_publisher.launch`, respectively |
| Mono-Inertial | [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)'s [`MH_01_easy.bag`](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag) | `roslaunch orb_slam3_ros euroc_mono_inertial.launch` <br /> `rosbag play MH_01_easy.bag --clock` | - |
| Mono-Inertial | [UniLu](#)'s [`Seq01.bag`](#) | `roslaunch orb_slam3_ros unilu_mono.launch` <br /> `rosbag play Seq01.bag --clock` | data collected by RealSense D435i using Spot (no `aligned_depth_to_color` and `IMU` available) |
| Stereo-Inertial | [TUM-VI](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)'s [`dataset-corridor1_512_16.bag`](https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-corridor1_512_16.bag) | `roslaunch orb_slam3_ros tum_vi_stereo_inertial.launch` <br /> `rosbag play dataset-corridor1_512_16.bag --clock` | - |
| Stereo-Inertial | Live (*) | `roslaunch realsense2_camera rs_t265.launch` <br /> `roslaunch orb_slam3_ros rs_t265_stereo_inertial.launch` | follow the hints (*) |
| RGB-D | [TUM](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)'s [`rgbd_dataset_freiburg1_xyz.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag) | `roslaunch orb_slam3_ros tum_rgbd.launch` <br /> `rosbag play rgbd_dataset_freiburg1_xyz.bag --clock` | change `TUMX.yaml` to `TUM1.yaml`,`TUM2.yaml` or `TUM3.yaml` for freiburg1, freiburg2 and freiburg3 sequences respectively. |
| RGB-D-Inertial | [VINS-RGBD](https://github.com/STAR-Center/VINS-RGBD)'s [`Normal.bag`](https://star-center.shanghaitech.edu.cn/seafile/d/0ea45d1878914077ade5/) | `roslaunch orb_slam3_ros rs_d435i_rgbd_inertial.launch` <br /> `rosbag play Normal.bag --clock` | decompress the downloaded bag using `rosbag decompress Normal.bag` and change the params in `RealSense_D435i.yaml` if necessary. |
| RGB-D-Inertial | Live (*) | `roslaunch orb_slam3_ros unilu_rgbd_inertial.launch` <br /> `roslaunch realsense2_camera rs_rgbd.launch align_depth:=true unite_imu_method:=linear_interpolation` | follow the hints (*) - use the sample modified file for RGB-D-Inertial device available [here](doc/realsense2_camera_rs_rgbd.launch) |

(*) For live mode, you need to first install `realsense-ros` using the instructions provided [here](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy), summarized as below:

```
# Catkin workspace folder
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/

# Cloning the latest code
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..

# Installing the library
catkin init
catkin build

# Sourcing the new configurations
# nano ~/.bashrc -> Add "alias sourcerealsense='source ~/workspace/realsense/rs_ros/devel/setup.bash'"
sourcerealsense

# Do a quick test
roslaunch realsense2_camera rs_rgbd.launch
```

Moreover, for using RGB-D cameras as the live feed provider, you may also require [rgbd_launch](http://wiki.ros.org/rgbd_launch) to load the nodelets to convert raw depth/RGB/IR streams to depth images. Otherwise, you may face a "resource not found" error.

## 💾 Data Collection

To record a `rosbag` file using a RealSense D435i camera and capture IMU, aligned depth, and color, you can follow these steps:

- Make sure you have the necessary drivers and packages installed for the RealSense camera to work with ROS, including `realsense2_camera` and `realsense2_description` packages using the following command:
- Launch the `realsense2_camera` node using the command `roslaunch realsense2_camera rs_aligned_depth.launch`
- Navigate to the directory where you want to save the rosbag file.
- Record the topics of interest. For instance, `rosbag record /camera/aligned_depth_to_color/image_raw /camera/color/image_raw /camera/imu /camera/aligned_depth_to_color/camera_info`

### Save and load map 

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).
#### Load map:
- Set the name of the map file to be loaded with `System.LoadAtlasFromFile` param in the settings file (`.yaml`).
- If the map file is not available, `System.LoadAtlasFromFile` param should be commented out otherwise there will be error.
#### Save map:
- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session
```
rosservice call /orb_slam3/save_map [file_name]
```

## 4. ROS topics, params and services
### Subscribed topics
- `/camera/image_raw` for Mono(-Inertial) node
- `/camera/left/image_raw` for Stereo(-Inertial) node
- `/camera/right/image_raw` for Stereo(-Inertial) node
- `/imu` for Mono/Stereo/RGBD-Inertial node
- `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` for RGBD node
### Published topics
- `/orb_slam3/camera_pose`, left camera pose in world frame, published at camera rate
- `/orb_slam3/body_odom`, imu-body odometry in world frame, published at camera rate
- `/orb_slam3/tracking_image`, processed image from the left camera with key points and status text
- `/orb_slam3/tracked_points`, all key points contained in the sliding window
- `/orb_slam3/all_points`, all key points in the map
- `/orb_slam3/kf_markers`, markers for all keyframes' positions
- `/tf`, with camera and imu-body poses in world frame
### Params
- `voc_file`: path to vocabulary file required by ORB-SLAM3
- `settings_file`: path to settings file required by ORB-SLAM3
- `enable_pangolin`: enable/disable ORB-SLAM3's Pangolin viewer and interface. (`true` by default)
- `enable_publish_static_tf_transform`: enable/disable static transform between two coordinate frames. (`false` by default, needs to be `true` for some datasets like `UniLu`)

### Services
- `rosservice call /orb_slam3/save_map [file_name]`: save the map as `[file_name].osa` in `ROS_HOME` folder.
- `rosservice call /orb_slam3/save_traj [file_name]`: save the estimated trajectory of camera and keyframes as `[file_name]_cam_traj.txt` and  `[file_name]_kf_traj.txt` in `ROS_HOME` folder.

### Docker
Provided [Dockerfile](Dockerfile) sets up an image based a ROS noetic environment including RealSense SDK

To access a USB device (such as RealSense camera) inside docker container use:
``` bash
docker run --network host --privileged -v /dev:/dev -it [image_name]
```

> **_NOTE:_**  `--network host` is recommended to listen to rostopics outside the container