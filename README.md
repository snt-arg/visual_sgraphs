# Visual S-Graphs

![Visual S-Graphs](demo.gif "Visual S-Graphs")

A marker-based VSLAM framework built on top of [ORB-SLAM 3.0](https://github.com/UZ-SLAMLab/VS_GRAPHS) (ROS implementation version introduced [here](https://github.com/thien94/orb_slam3_ros)) that supports adding semantic entities to the final map and adding hierarchical representations.

[![arXiv](https://img.shields.io/badge/arXiv-2309.10461-b31b1b.svg)](https://arxiv.org/abs/2309.10461)

## 📃 Table of Content

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configurations](#configurations)
- [Run Examples](#run)
- [Data Collection](#data)
- [ROS Topics, Params and Services](#ros)
- [Maps](#maps)
- [Evaluation](#eval)
- [TODO](#todo)

## 📝 Prerequisites <a id="prerequisites"></a>

Install the required libraries listed below:

### OpenCV <a id="opencv"></a>

Check the OpenCV version on your computer (required [at least 3.0](https://github.com/UZ-SLAMLab/VS_GRAPHS)):

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

Please refer to [this page](/doc/RealSense/README.md) for detailed description on how to prepare a RealSense D400 series camera for live feed or data collection.

### 🎨 Kimera-Semantics (optional) <a id="kimera"></a>

Install `Kimera-Semantics` based on the installation guide introduced [here](https://github.com/MIT-SPARK/Kimera-Semantics/tree/master). In case you have `Ros Noetic`, you may face some errors related to `pcl` library and the build fails. In this case, you should apply `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14` to be able to build it ([issue](https://github.com/MIT-SPARK/Kimera-Semantics/issues/67)).

### `hector-trajectory-server` (optional)

Using this library you can visualize the real-time trajectory of `camera/IMU`.

```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

## ⚙️ Installation <a id="installation"></a>

After installing the prerequisites, you can install the repository using commands below:

### I. Cloning the Repository <a id="cloning"></a>

You should first create a workspace and clone the Visual S-Graphs repository in it:

```
cd ~/[workspace]/src
git clone git@github.com:snt-arg/visual_sgraphs.git
```

### II. Cloning the `aruco_ros` Repository <a id="aruco"></a>

This package (available [here](https://github.com/pal-robotics/aruco_ros)) enables you to detect ArUco Markers in cameras' field of view. Accordingly, install it using the commands below in **the same folder (i.e., [workspace]/src)**:

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone -b noetic-devel git@github.com:pal-robotics/aruco_ros.git
```

It is important to put the file in the same folder, as the Visual S-Graphs library depends on it. Instead of the original launch file, you can use the sample modified `marker_publisher.launch` file for this library available [here](doc/aruco_ros_marker_publisher.launch), which works fine with the _UniLu_ dataset and the live feed for RealSense cameras (`imageRaw` and `cameraInfo` should be changed based on the use case). Do not forget to set proper `ref_frame`, `markerSize`, `imageRaw`, and `cameraInfo` values in the launch file.

### III. Cloning the `scene_segment_ros` Repository <a id="segmenter"></a>

This package (available [here](https://github.com/snt-arg/scene_segment_ros)) enables you to **segment the scene** and **detect semantic objects** in the scene. Accordingly, install it using the commands below in **the same folder (i.e., [workspace]/src)**:

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone git@github.com:snt-arg/scene_segment_ros.git
```

It is important to put the file in the same folder, as the Visual S-Graphs library depends on it. You can then run the scene semantic segmentor using the commands `roslaunch segmenter_ros segmenter_pFCN.launch` for using the **PanopticFCN** model. Reade more about available options in the [repo](https://github.com/snt-arg/scene_segment_ros).

### IV. 🦊 Installing Voxblox Skeleton <a id="voxblox"></a>

This package (available [here](https://github.com/snt-arg/mav_voxblox_planning/tree/master)) enables you to use `voxblox` and `loco planning` for cluster-based room detection. Accordingly, install it using the commands below in a workspace **not necessarily the same folder (i.e., [workspace]/src)**:

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone git@github.com:snt-arg/mav_voxblox_planning.git
wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall
wstool update
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build
```

### V. Installing Other Required Libraries <a id="libraries"></a>

First, make sure that you have installed all the required dependencies, such as `ros-noetic-backward-ros` and `ros-noetic-rviz-visual-tools`, using the command `rosdep install --from-paths src --ignore-src -y`.

### VI. Build

Build the installed libraries and modules using `catkin build`. As a shortcut, you can add a new alias to the `bashrc` file to run the environment whenever needed, like below:

```
alias sourceros='source /opt/ros/noetic/setup.bash'
alias sourcevox="source ~/workspace/ros/voxblox_skeleton/devel/setup.bash"
alias sourcerealsense='source ~/workspace/realsense/rs_ros/devel/setup.bash'
alias sourcevsgraphs='source ~/workspace/ros/orbslam3_ros_ws/devel/setup.bash'
```

As a quick test, you can do as follows:

- Run a `roscore`
- Run the `aruco_ros` using `sourcevsgraphs` and then `roslaunch aruco_ros marker_publisher.launch [2>/dev/null]` for detecting multiple markers in the scene and publishing their poses.
  - The final results (scene with detected markers) produced by `aruco_ros` are published and accessible on `/aruco_marker_publisher/result` and the pose of the markers will be shown using `rostopic echo /aruco_marker_publisher/markers`.
- Run the Visual S-Graphs using `sourcevsgraphs` and then `roslaunch orb_slam3_ros vsgraphs_rgbd.launch [2>/dev/null]`

## 🔨 Configurations <a id="configurations"></a>

You can find the configuration files for the application in the `config` folder. It contains some `Calibration` processes, camera parameter for various sensors, and some `rviz` files for different datasets. You can define your own `rviz` and `yaml` files according to the characteristics of the sensor you collected data with. A sample of how to set camera intrinsic and extrinsic parameters can be found [here](https://github.com/shanpenghui/ORB_SLAM3_Fixed#73-set-camera-intrinsic--extrinsic-parameters).

### ⚙️ Common System Parameters

You can read about the configuration parameters (independent of the `ROS` wrapper) for performing SLAM [here](/config/README.md). They can be find and modified in [common_system_params.yaml](/config/common_system_params.yaml).

## 🚀 Run Examples <a id="run"></a>

1. You can download some sample dataset instances from the links provided below and run them using `rosbag play [sample].bag --clock [-s x]`:

   - UniLu's single office ([link](https://uniluxembourg-my.sharepoint.com/:u:/r/personal/ali_tourani_uni_lu/Documents/Data/ULMS-Seq06.zip?csf=1&web=1&e=vyBNPZ)) for Mono, Mono-Inertial, RGB-D, RGB-D Inertial
   - UniLu's single office ([link](https://uniluxembourg-my.sharepoint.com/:u:/r/personal/ali_tourani_uni_lu/Documents/Data/ULMS-Seq06.zip?csf=1&web=1&e=vyBNPZ)) for Stereo and Stereo-Inertial

2. Run the ArUco marker detector module using `roslaunch aruco_ros marker_publisher.launch`
3. Run the Semantic Segmentation module (pFCN) using `roslaunch segmenter_ros segmenter_pFCN.launch`
4. Run VoxBlox Skeleton using `roslaunch voxblox_skeleton skeletonize_map_vsgraphs.launch 2>/dev/null`

| Mode            | Dataset                            | Commands                                                                   | Notes                          |
| --------------- | ---------------------------------- | -------------------------------------------------------------------------- | ------------------------------ |
| Mono            | UniLu's single office or Live (\*) | `roslaunch orb_slam3_rosvsgraphsmono.launch [offline:=false]`              | data collected using RealSense |
| Mono-Inertial   | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros vsgraphs_mono_inertial.launch [offline:=false]`   | data collected using RealSense |
| Stereo          | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros vsgraphs_stereo.launch [offline:=false]`          | data collected using RealSense |
| Stereo-Inertial | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros vsgraphs_stereo_inertial.launch [offline:=false]` | data collected using RealSense |
| RGB-D           | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros vsgraphs_rgbd.launch [offline:=false]`            | data collected using RealSense |
| RGB-D-Inertial  | UniLu's single office or Live (\*) | `roslaunch orb_slam3_ros vsgraphs_rgbd_inertial.launch [offline:=false]`   | data collected using RealSense |

### ⚠️ Useful Hints <a id="hints"></a>

#### 🦊 Voxblox Integration <a id="voxblox-integrate"></a>

For detecting rooms, you need to use `voxblox skeleton` instead of the normal version of `voxblox`, as it uses free spaces for clustering in `skeletonize_map_vsgraphs` launch file.

If you want to use normal `voxblox` (faces challenges for room creation), you may need to first create a launch file that can be integrated into this framework. You can find a sample of such launch file [here](doc/voxblox_rs_rgbd.launch). Then, for running `voxblox`, you need to source it and run it in a separate terminal using `roslaunch voxblox_ros vsgraphs_rgbd.launch`.

Additionally, before running the framework, you need to source it, source `voxblox` with a `--extend` command, and then launch the framework.

```
source /opt/ros/noetic/setup.bash &&
source ~/[VSGRAPHS_PATH]/devel/setup.bash &&
source ~/[VOXBLOX_PATH]/devel/setup.bash --extend &&
roslaunch orb_slam3_ros vsgraphs_rgbd.launch 2>/dev/null
```

[Note] As `voxblox` and `Visual S-Graphs` both need to access/modify `TF` data, it may become slow. So, in order to run it with less computation cost and avoid chunking the reconstructed map, you may need to:

- Use the command `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release` in `voxblox`'s workspace to build it in the release mode and run it again,
- Run the rosbag file slower using `rosbag play [file] --clock -r 0.5`

#### ArUco Detector Integration <a id="aruco-integrate"></a>

Note that given the input sensor (Mono, RGB-D, and Stereo), the parameters fed to `aruco_ros` varies. Regarding the sample launch file available [here](doc/aruco_ros_marker_publisher.launch), we can use the library as below:

- For Mono and RGB-D feed use `roslaunch aruco_ros marker_publisher.launch`,
- For Stereo feed use `roslaunch aruco_ros marker_publisher.launch imageRaw:=/camera/infra1/image_rect_raw 
cameraInfo:=/camera/infra1/camera_info`

#### 📸 Live Version <a id="live"></a>

1. For the live version, you need to add the flag `offline:=false`, such as `roslaunch orb_slam3_ros vsgraphs_mono.launch offline:=false`, which gets the **TF** values from RealSense instead of reading from `ORB-SLAM` while a `rosbag` file is being played (offline). If the flag is not set, whenever a marker is seen, the tracking will fail due to the mentioned conflict.
2. The next step is to choose among different setups:

   - For RGB-D cameras as the live feed provider, you may also require [rgbd_launch](http://wiki.ros.org/rgbd_launch) to load the nodelets to convert raw depth/RGB/IR streams to depth images. Otherwise, you may face a "resource not found" error. You can find a sample launch file for RGB-D and Mono [here](/doc/realsense2_camera_rs_rgbd.launch).
   - For using Stereo cameras as the live feed provider, you can find a sample launch file [here](/doc/realsense2_camera_rs_stereo.launch).
     - Note that you have to stop the **emitter** in RealSense. There are some arguments in the launch file, but changing the does not work do the job. So, the easiest solution is to run `realsense-viewer` and set `Emitter Enabled` to False.

3. Run realsense use the command `roslaunch realsense2_camera [rs_rgbd/rs_stereo].launch [align_depth:=true] [unite_imu_method:=linear_interpolation]`.

#### 🔖 Using IMU <a id="imu"></a>

Please note that in order to use inertial sensors (i.e., _IMU_) you need to initialize it first. As you can see in the animation below, the _IMU_ needs to move steady forward and backward for around 10 seconds while facing a scene with lots of visual features. The logs appeared in the console will show if the _IMU_ is initialized or not.

![IMU Initialization](demo-IMU.gif "IMU Initialization")

## 💾 Data Collection <a id="data"></a>

### I. Using a RealSense Camera

Please refer to [this page](/doc/RealSense/README.md) for detailed description on how to use a RealSense D400 series camera for data collection.

### II. Using the Handheld Device

To record a `rosbag` file using the **Handheld Device** developed by the team, containing a RealSense D435i and a 3D LiDAR, you need to follow the below steps:

- Create a `SSH` file to connect to the device. You just need to add the ssh information to the `config` file available in your `.ssh` folder. There, you need to add the following settings:

```
Host unitree
    Hostname 10.42.0.1
    User unitree-brain
```

- Connect to `unitree`'s Wi-Fi and run `ssh unitree`. You need to run the same thing for two `bash` areas. You will be connected to the `bash` control of the device.
  - [Note] You need to have the password to connect.
- In the first command area run `neurolink_mprocs`, so that you have access to all the commands required to run services. Run them all one by one.
- In the second command, run the service provided for `rosbag` recording, which is `./rosbag_collection.sh .`. Note that the second dot is the location to save the rosbag file.
- You can always connect to `unitree`'s Wi-Fi and check the address `10.42.0.1:8888` to see all the topics provided by the software.

## 🤖 ROS Topics, Params and Services <a id="ros"></a>

### Subscribed Topics <a id="ros-sub"></a>

| Topic                                                            | Description                                           |
| ---------------------------------------------------------------- | ----------------------------------------------------- |
| `/imu`                                                           | for Mono/Stereo/RGBD-Inertial node                    |
| `/camera/image_raw`                                              | for Mono(-Inertial) node                              |
| `/camera/left/image_raw` and `/camera/right/image_raw`           | for Stereo(-Inertial) node                            |
| `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` | for RGBD node                                         |
| `/aruco_marker_publisher/markers`                                | for ArUco marker library node                         |
| `/camera/color/image_segment`                                    | for Semantic segmenter library node (custom message)  |
| `/camera/color/image_segment_vis`                                | for Semantic segmenter library node (segmented image) |

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
| `/orb_slam3/keyframe_image`   | keyframe poses images                                                |
| `/orb_slam3/fiducial_markers` | fiducial markers detected in the environment                         |
| `/orb_slam3/doors`            | doorways detected in the environment                                 |
| `/orb_slam3/planes`           | planes detected in the environment                                   |
| `/orb_slam3/rooms`            | corridors and rooms markers detected in the environment              |

### Params <a id="ros-param"></a>

| Param                                                     | Description                                                                                                    |
| --------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `offline`                                                 | live or reading rosbag file (offline)?                                                                         |
| `sys_params_file`                                         | path to the common system parameters (see below)                                                               |
| `voc_file`                                                | path to ORB vocabulary file                                                                                    |
| `settings_file`                                           | path to settings file                                                                                          |
| `enable_pangolin`                                         | enable/disable Pangolin viewer and interface. (`true` by default)                                              |
| `static_transform`                                        | enable/disable static transform between coordinate frames. (needs to be `true` for some datasets like `UniLu`) |
| `roll`, `yaw`, and `pitch`                                | poses and dimensions of movement                                                                               |
| `frame_map` <br /> `world_frame_id` <br /> `cam_frame_id` | different frame identifiers                                                                                    |

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
     - Declare the file path to be saved, the slame method you want to obtain the poses, the dataset name, etc. before running it. For instance, if `slam_pose_semorb3_seq01.txt` is generated, it means it contain the SLAM poses of `Visual S-Graphs` on `Seq01` dataset instance.
     - It will create a new `txt` file, and the poses will be added there.
  2. Run the framework you need for the evaluation:
     - For `S-Graphs`, the file is generated while running it on a rosbag file.
     - For `ORB-SLAM 3.0 (ROS version)` and `Visual S-Graphs (current repo)`, they need to be run and a rosbag should be played to fill the `txt` file.
     - For `UcoSLAM` and `Semantic UcoSLAM`, we need to have the poses created in a new way. Accordingly, we need to put the poses created by the `S-Graphs` in `/tmp/timestamp.txt`, and then run the codes from the [ros-wrapper](https://github.com/snt-arg/ucoslam_ros_wrapper/tree/main/src):
       - UcoSLAM: `rosrun ucoslam_ros ucoslam_ros_video /[PATH]/vid.mp4 [PATH]/ucoslam_ros_wrapper/config/realsense_color_640_480_spot.yml -aruco-markerSize 0.78 -dict ARUCO -voc [PATH]/ucoslam_ros_wrapper/config/orb.fbow`
       - Semantic UcoSLAM: `rosrun ucoslam_ros ucoslam_ros_semantics_video /[PATH]/vid.mp4 [PATH]/ucoslam_ros_wrapper/config/realsense_color_640_480_spot.yml -aruco-markerSize 0.78 -dict ARUCO -voc [PATH]/ucoslam_ros_wrapper/config/orb.fbow`
  3. Finally, when the ground-truth (S-Graphs) and SLAM pose (e.g., UcoSLAM, etc.) are ready, you can use the [`evo_ape`](https://github.com/MichaelGrupp/evo) for evaluation, like `evo_ape tum s_graphs_pose_seq05.txt slam_pose_semuco_seq05.txt -va > results.txt --plot --plot_mode xy`

## 🗒️ TODO <a id="todo"></a>

Here is the list of TODO tasks that can be integrated in the project:

- Due to the sparse pointclouds in Mono and Stereo, calculate the depth from points using Machine Learning to get a better plane estimate (`Tracking::getPlanesFromPointClouds()`).
- Subscribing to the robot's odometry for not getting lost
- Semantic loop closure detection based on high-level entities
- Change `common.h` to a Class and set the values in `ros_[x].cc`
- Create a yaml config file for common launch parameters `vsgraphs_ros_[x].launch`
