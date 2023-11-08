# Visual S-Graphs

![Visual S-Graphs](demo.gif "Visual S-Graphs")

A marker-based VSLAM framework built on top of [ORB-SLAM 3.0](https://github.com/UZ-SLAMLab/ORB_SLAM3) (ROS implementation version introduced [here](https://github.com/thien94/orb_slam3_ros)) that supports adding semantic entities to the final map and adding hierarchical representations.

## 📃 Table of Content

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configurations](#configurations)
- [Run Examples](#run)
- [Data Collection](#data)
- [ROS Topics, Params and Services](#ros)
- [Maps](#maps)
- [Evaluation](#eval)

## 📝 Prerequisites <a id="prerequisites"></a>

Install the required libraries listed below:

### OpenCV <a id="opencv"></a>

Check the OpenCV version on your computer (required [at least 3.0](https://github.com/UZ-SLAMLab/ORB_SLAM3)):

```
python3 -c "import cv2; print(cv2.__version__)"
```

On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is already included. If a newer version is required (>= 3.0), follow [installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) and change the corresponding OpenCV version in `CMakeLists.txt`

### Eigen3 <a id="eigen"></a>

Install `Eigen`, which is a C++ template library for linear algebra (including matrices, vectors, and numerical solvers):

```
sudo apt install libeigen3-dev
```

### Pangolin <a id="pangolin"></a>

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

### 🎞️ RealSense (Live Mode - optional) <a id="realsense"></a>

For running in live mode, you need to first install `realsense-ros` using the instructions provided [here](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy), summarized as below:

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
roslaunch realsense2_camera rs_rgbd.launch [2>/dev/null]
```

### 🦊 Voxblox (optional) <a id="voxblox"></a>

Install `Voxblox` based on the installation guide introduced [here](https://voxblox.readthedocs.io/en/latest/pages/Installation.html), and to make sure if it works fine, try [running it](https://voxblox.readthedocs.io/en/latest/pages/Running-Voxblox.html) on a simple dataset, such as the `basement dataset`.

### `hector-trajectory-server` (optional)

Using this library you can visualize the real-time trajectory of `camera/IMU`.

```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

## ⚙️ Installation <a id="installation"></a>

After installing the prerequisites, you can install the repository using commands below:

### I. Cloning the Repository <a id="cloning"></a>

You should first create a workspace and clone the Semantic ORB-SLAM 3.0 repository in it:

```
cd ~/[workspace]/src
git clone git@github.com:snt-arg/semantic_orb_slam3_ros.git
```

### II. Cloning the `aruco_ros` Repository <a id="aruco"></a>

This package (available [here](https://github.com/pal-robotics/aruco_ros)) enables you to detect ArUco Markers in cameras' field of view. Accordingly, install it using the commands below in **the same folder (i.e., [workspace]/src)**:

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone git@github.com:pal-robotics/aruco_ros.git
```

It is important to put the file in the same folder, as the Semantic ORB-SLAM 3.0 library depends on it. Instead of the original launch file, you can use the sample modified `marker_publisher.launch` file for this library available [here](doc/aruco_ros_marker_publisher.launch), which works fine with the _UniLu_ dataset and the live feed for RealSense cameras (`imageRaw` and `cameraInfo` should be changed based on the use case). Do not forget to set proper `ref_frame`, `markerSize`, `imageRaw`, and `cameraInfo` values in the launch file.

### III. Installing the Libraries <a id="libraries"></a>

Install both the libraries using `catkin build`. Finally, you can add a new alias to the `bashrc` file to run the environment whenever needed:

```
alias sourceorb3ros='source ~/workspace/ros/orbslam3_ros_ws/devel/setup.bash'
```

As a quick test, you can do as follows:

- Run a `roscore`
- Run the `aruco_ros` using `sourceorb3ros` and then `roslaunch aruco_ros marker_publisher.launch [2>/dev/null]` for detecting multiple markers in the scene and publishing their poses.
  - The final results (scene with detected markers) produced by `aruco_ros` are published and accessible on `/aruco_marker_publisher/result` and the pose of the markers will be shown using `rostopic echo /aruco_marker_publisher/markers`.
- Run the Semantic ORB-SLAM using `sourceorb3ros` and then `roslaunch orb_slam3_ros unilu_rgbd.launch [2>/dev/null]`

## 🔨 Configurations <a id="configurations"></a>

You can find the configuration files for the application in the `config` folder. It contains some `Calibration` processes, camera parameter for various sensors, and some `rviz` files for different datasets. You can define your own `rviz` and `yaml` files according to the characteristics of the sensor you collected data with. A sample of how to set camera intrinsic and extrinsic parameters can be found [here](https://github.com/shanpenghui/ORB_SLAM3_Fixed#73-set-camera-intrinsic--extrinsic-parameters).

## 🚀 Run Examples <a id="run"></a>

1. You can download some sample dataset instances from the links provided below and run them using `rosbag play [sample].bag --clock [-s x]`:

   - UniLu's single office ([link](https://uniluxembourg-my.sharepoint.com/:u:/r/personal/ali_tourani_uni_lu/Documents/Data/ULMS-Seq06.zip?csf=1&web=1&e=vyBNPZ)) for Mono, Mono-Inertial, RGB-D, RGB-D Inertial
   - UniLu's single office ([link](https://uniluxembourg-my.sharepoint.com/:u:/r/personal/ali_tourani_uni_lu/Documents/Data/ULMS-Seq06.zip?csf=1&web=1&e=vyBNPZ)) for Stereo and Stereo-Inertial

2. Run the ArUco marker detector module using `roslaunch aruco_ros marker_publisher.launch`

| Mode            | Dataset                            | Commands                                                                | Notes                          |
| --------------- | ---------------------------------- | ----------------------------------------------------------------------- | ------------------------------ |
| Mono            | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros unilu_mono.launch [offline:=false]`            | data collected using RealSense |
| Mono-Inertial   | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros unilu_mono_inertial.launch [offline:=false]`   | data collected using RealSense |
| Stereo          | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros unilu_stereo.launch [offline:=false]`          | data collected using RealSense |
| Stereo-Inertial | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros unilu_stereo_inertial.launch [offline:=false]` | data collected using RealSense |
| RGB-D           | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros unilu_rgbd.launch [offline:=false]`            | data collected using RealSense |
| RGB-D-Inertial  | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros unilu_rgbd_inertial.launch [offline:=false]`   | data collected using RealSense |

### ⚠️ Useful Hints <a id="hints"></a>

#### 🦊 Voxblox Integration <a id="voxblox-integrate"></a>

You need to first create a launch file that can be integrated into this framework. You can find a sample of such launch file [here](doc/voxblox_rs_rgbd.launch). Then, for running `voxblox`, you need to source it and run it in a separate terminal using `roslaunch voxblox_ros unilu_rgbd.launch`.

Additionally, before running the framework, you need to source it, source `voxblox` with a `--extend` command, and then launch the framework.

```
source /opt/ros/noetic/setup.bash &&
source ~/[VSGRAPHS_PATH]/devel/setup.bash &&
source ~/[VOXBLOX_PATH]/devel/setup.bash --extend &&
roslaunch orb_slam3_ros unilu_rgbd.launch 2>/dev/null
```

[Note] As `voxblox` and `Visual S-Graphs` both need to access/modify `TF` data, it may become slow. So, in order to run it with less computation cost and avoid chunking the reconstructed map, you may need to:

- Use the command `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release` in `voxblox`'s workspace to build it in the release mode and run it again,
- Run the rosbag file slower using `rosbag play [file] --clock -r 0.5`

#### 📸 Live Version <a id="imu"></a>

1. For using the live version, you need to add the flag `offline:=false`, such as `roslaunch orb_slam3_ros unilu_mono.launch offline:=false`, which gets the **TF** values from RealSense instead of reading from `ORB-SLAM` while a `rosbag` file is being played (offline). If the flag is not set, whenever a marker is seen, the tracking will fail due to the mentioned conflict.
2. For using RGB-D cameras as the live feed provider, you may also require [rgbd_launch](http://wiki.ros.org/rgbd_launch) to load the nodelets to convert raw depth/RGB/IR streams to depth images. Otherwise, you may face a "resource not found" error. You can find a sample launch file for RGB-D and Mono [here](/doc/realsense2_camera_rs_rgbd.launch).
3. For using Stereo cameras as the live feed provider, you can find a sample launch file [here](/doc/realsense2_camera_rs_stereo.launch).
4. Run realsense using `roslaunch realsense2_camera [rs_rgbd/rs_stereo].launch [align_depth:=true] [unite_imu_method:=linear_interpolation]`.

#### 🔖 Using IMU <a id="imu"></a>

Please note that in order to use inertial sensors (i.e., _IMU_) you need to initialize it first. As you can see in the animation below, the _IMU_ needs to move steady forward and backward for around 10 seconds while facing a scene with lots of visual features. The logs appeared in the console will show if the _IMU_ is initialized or not.

![IMU Initialization](demo-IMU.gif "IMU Initialization")

## 💾 Data Collection <a id="data"></a>

To record a `rosbag` file using a **RealSense D435i** camera and capture _IMU_, _aligned depth_, _stereo_, and _color_, you can follow these steps:

- Make sure you have the necessary drivers and packages installed for the RealSense camera to work with `ROS`, including `realsense2_camera` and `realsense2_description` packages using the following command:
- Launch the `realsense2_camera` node using the proper command:
  - For Mono and RGB-D, with or without IMU, run `roslaunch realsense2_camera rs_rgbd.launch` (sample provided [here](/doc/realsense2_camera_rs_rgbd.launch)),
  - For Mono and RGB-D, with or without IMU, run `roslaunch realsense2_camera rs_stereo.launch` (sample provided [here](/doc/realsense2_camera_rs_stereo.launch)),
- Navigate to the directory where you want to save the rosbag file,
- Record the topics of interest:
  - For Mono and RGB-D, with or without IMU, run `rosbag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/imu /camera/color/camera_info /camera/aligned_depth_to_color/camera_info`.
  - For Stereo, with or without IMU, run `rosbag record /camera/color/image_raw /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu /camera/color/camera_info /camera/infra1/camera_info /camera/infra2/camera_info`.

## 🤖 ROS Topics, Params and Services <a id="ros"></a>

### Subscribed Topics <a id="ros-sub"></a>

| Topic                                                            | Description                        |
| ---------------------------------------------------------------- | ---------------------------------- |
| `/imu`                                                           | for Mono/Stereo/RGBD-Inertial node |
| `/camera/image_raw`                                              | for Mono(-Inertial) node           |
| `/camera/left/image_raw` and `/camera/right/image_raw`           | for Stereo(-Inertial) node         |
| `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` | for RGBD node                      |
| `/aruco_marker_publisher/markers`                                | for ArUco marker library node      |

### Published Topics <a id="ros-pub"></a>

| Topic                         | Description                                                          |
| ----------------------------- | -------------------------------------------------------------------- |
| `/tf`                         | with camera and imu-body poses in world frame                        |
| `/orb_slam3/camera_pose`      | left camera pose in world frame, published at camera rate            |
| `/orb_slam3/body_odom`        | imu-body odometry in world frame, published at camera rate           |
| `/orb_slam3/tracking_image`   | processed image from the left camera with key points and status text |
| `/orb_slam3/tracked_points`   | all key points contained in the sliding window                       |
| `/orb_slam3/all_points`       | all key points in the map                                            |
| `/orb_slam3/kf_markers`       | markers for all keyframes' positions                                 |
| `/orb_slam3/fiducial_markers` | fiducial markers detected in the environment                         |
| `/orb_slam3/doors`            | doorways detected in the environment                                 |
| `/orb_slam3/walls`            | walls detected in the environment                                    |
| `/orb_slam3/rooms`            | corridors and rooms markers detected in the environment              |

### Params <a id="ros-param"></a>

| Param                                                        | Description                                                                                                    |
| ------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------- |
| `offline`                                                    | live or reading rosbag file (offline)?                                                                         |
| `env_database`                                               | semantic map data file to be loaded                                                                            |
| `voc_file`                                                   | path to ORB vocabulary file                                                                                    |
| `settings_file`                                              | path to settings file                                                                                          |
| `enable_pangolin`                                            | enable/disable Pangolin viewer and interface. (`true` by default)                                              |
| `publish_static_transform`                                   | enable/disable static transform between coordinate frames. (needs to be `true` for some datasets like `UniLu`) |
| `roll`, `yaw`, and `pitch`                                   | poses and dimensions of movement                                                                               |
| `map_frame_id` <br /> `world_frame_id` <br /> `cam_frame_id` | different frame identifiers                                                                                    |

## 📍 Maps <a id="maps"></a>

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).

### Load Map <a id="maps-load"></a>

- Set the name of the map file to be loaded with `System.LoadAtlasFromFile` param in the settings file (`.yaml`).
- If the map file is not available, `System.LoadAtlasFromFile` param should be commented out otherwise there will be error.

### Save Map <a id="maps-save"></a>

- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session

```
rosservice call /orb_slam3/save_map [file_name]
```

### Services <a id="maps-services"></a>

- `rosservice call /orb_slam3/save_map [file_name]`: save the map as `[file_name].osa` in `ROS_HOME` folder.
- `rosservice call /orb_slam3/save_traj [file_name]`: save the estimated trajectory of camera and keyframes as `[file_name]_cam_traj.txt` and `[file_name]_kf_traj.txt` in `ROS_HOME` folder.

## 📊 Evaluation <a id="eval"></a>

In order to evaluate the current method with others, such as UcoSLAM, ORB-SLAM 3.0, etc., you need to follow the below instructions:

- Prepare the `.txt` file containing robot poses, generated using a framework:
  1. Run `generate_pose_txt_files.py` in the `evaluation` folder of this repository.
     - Declare the file path to be saved, the slame method you want to obtain the poses, the dataset name, etc. before running it. For instance, if `slam_pose_semorb3_seq01.txt` is generated, it means it contain the SLAM poses of `Semantic ORB-SLAM 3.0` on `Seq01` dataset instance.
     - It will create a new `txt` file, and the poses will be added there.
  2. Run the framework you need for the evaluation:
     - For `S-Graphs`, the file is generated while running it on a rosbag file.
     - For `ORB-SLAM 3.0 (ROS version)` and `Semantic ORB-SLAM 3.0 (current repo)`, they need to be run and a rosbag should be played to fill the `txt` file.
     - For `UcoSLAM` and `Semantic UcoSLAM`, we need to have the poses created in a new way. Accordingly, we need to put the poses created by the `S-Graphs` in `/tmp/timestamp.txt`, and then run the codes from the [ros-wrapper](https://github.com/snt-arg/ucoslam_ros_wrapper/tree/main/src):
       - UcoSLAM: `rosrun ucoslam_ros ucoslam_ros_video /[PATH]/vid.mp4 [PATH]/ucoslam_ros_wrapper/config/realsense_color_640_480_spot.yml -aruco-markerSize 0.78 -dict ARUCO -voc [PATH]/ucoslam_ros_wrapper/config/orb.fbow`
       - Semantic UcoSLAM: `rosrun ucoslam_ros ucoslam_ros_semantics_video /[PATH]/vid.mp4 [PATH]/ucoslam_ros_wrapper/config/realsense_color_640_480_spot.yml -aruco-markerSize 0.78 -dict ARUCO -voc [PATH]/ucoslam_ros_wrapper/config/orb.fbow`
  3. Finally, when the ground-truth (S-Graphs) and SLAM pose (e.g., UcoSLAM, etc.) are ready, you can use the [`evo_ape`](https://github.com/MichaelGrupp/evo) for evaluation, like `evo_ape tum s_graphs_pose_seq05.txt slam_pose_semuco_seq05.txt -va > results.txt --plot --plot_mode xy`
