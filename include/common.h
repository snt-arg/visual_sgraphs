#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <segmenter_ros/VSGraphDataMsg.h>
#include <segmenter_ros/SegmenterDataMsg.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <orb_slam3_ros/SaveMap.h>

// Transformation process
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"
#include "Types/SystemParams.h"

// ArUco-ROS library
#include <aruco_msgs/MarkerArray.h>

// Semantics
#include "Semantic/Door.h"
#include "Semantic/Room.h"
#include "Semantic/Marker.h"

using json = nlohmann::json;

class ORB_SLAM3::SystemParams;
extern ORB_SLAM3::System *pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;

extern double roll, pitch, yaw;       // Defining axes for transformation
extern bool publish_static_transform; // If true, it should use transformed calculations
extern std::string world_frame_id, cam_frame_id, imu_frame_id, map_frame_id, struct_frame_id, room_frame_id;

// List of visited Fiducial Markers in different timestamps
extern std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;

// List of white space cluster points obtained from `voxblox_skeleton`
extern std::vector<std::vector<Eigen::Vector3d *>> skeletonClusterPoints;

extern ros::Publisher kf_img_pub;
extern image_transport::Publisher tracking_img_pub;
extern ros::Publisher pose_pub, odom_pub, kf_markers_pub;
extern ros::Publisher tracked_mappoints_pub, all_mappoints_pub, segmented_pointclouds_pub;

struct MapPointStruct
{
    Eigen::Vector3f coordinates;
    int cluster_id;

    MapPointStruct(Eigen::Vector3f coords) : coordinates(coords), cluster_id(-1) {}
};

void setupServices(ros::NodeHandle &, std::string);
void publishTopics(ros::Time, Eigen::Vector3f = Eigen::Vector3f::Zero());
void setupPublishers(ros::NodeHandle &, image_transport::ImageTransport &, std::string);

void publishTrackingImage(cv::Mat, ros::Time);
void publishCameraPose(Sophus::SE3f, ros::Time);
void publishStaticTfTransform(string, string, ros::Time);
void publishRooms(std::vector<ORB_SLAM3::Room *>, ros::Time);
void publishDoors(std::vector<ORB_SLAM3::Door *>, ros::Time);
void publishPlanes(std::vector<ORB_SLAM3::Plane *>, ros::Time);
void publishTfTransform(Sophus::SE3f, string, string, ros::Time);
void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);
void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);
void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *>, ros::Time);
void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *>, ros::Time);
void publishKeyframeImages(std::vector<ORB_SLAM3::KeyFrame *>, ros::Time);
void publishKeyframeMarkers(std::vector<ORB_SLAM3::KeyFrame *>, ros::Time);
void publishBodyOdometry(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, ros::Time);

bool saveMapService(orb_slam3_ros::SaveMap::Request &, orb_slam3_ros::SaveMap::Response &);
bool saveTrajectoryService(orb_slam3_ros::SaveMap::Request &, orb_slam3_ros::SaveMap::Response &);

cv::Mat SE3f_to_cvMat(Sophus::SE3f);
tf::Transform SE3f_to_tfTransform(Sophus::SE3f);
sensor_msgs::PointCloud2 mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);

/**
 * @brief Adds the markers to the buffer to be processed
 * @param markerArray The array of markers received from `aruco_ros`
 */
void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray);

/**
 * @brief Avoids adding duplicate markers to the buffer by checking the timestamp
 * @param frameTimestamp The timestamp of the frame that captured the marker
 */
std::pair<double, std::vector<ORB_SLAM3::Marker *>> findNearestMarker(double frameTimestamp);

/**
 * @brief Gets skeleton voxels from `voxblox_skeleton` to be processed
 * @param skeletonArray The array of skeleton voxels received
 */
void getVoxbloxSkeleton(const visualization_msgs::MarkerArray &skeletonArray);