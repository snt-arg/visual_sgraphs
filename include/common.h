/**
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <limits>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
// #include <tf/transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/time.h> // for tf2::durationFromSec

#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For doTransform
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Make sure this include is present
// #include <tf2/tf2.h>
#include <functional>

// #include <std_msgs/Header.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <pcl_ros/common/common.hpp>
// #include <pcl/PCLPointCloud2.hpp>
// #include <pcl/common/distances.hpp>
// #include <pcl/filters/voxel_grid.hpp>
// Tranfororm previos pcl headers for ros2
#include <pcl_ros/transforms.hpp> // From your pcl_ros/ folder
// #include <pcl_ros/point_cloud.hpp>    // From your pcl_ros/ folder (if needed)
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/pcl_node.hpp> // From your pcl_ros/ folder (if needed)

// #include <sensor_msgs/PointCloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <visualization_msgs/Marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <segmenter_ros/msg/vs_graph_data_msg.hpp>
#include <segmenter_ros/msg/segmenter_data_msg.hpp>
#include <keyframe_depth_estimator/msg/key_frame_created.hpp>
#include <keyframe_depth_validator/msg/static_map_point_correspondences.hpp>
#include <keyframe_depth_validator/msg/static_map_points.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

// This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <vs_graphs/srv/save_map.hpp>

// Transformation process
#include <pcl_ros/transforms.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"
#include "Types/SystemParams.h"

// ArUco-ROS library
// #include <aruco_msgs/MarkerArray.h>

// Semantics
#include "Semantic/Door.h"
#include "Semantic/Room.h"
#include "Semantic/Marker.h"

// Situational Graphs Messages
#include <situational_graphs_msgs/msg/rooms_data.hpp>
#include <situational_graphs_msgs/msg/planes_data.hpp>

// vS-Graphs Custom Messages
#include <vs_graphs/msg/vs_graphs_all_walls_data.hpp>
#include <vs_graphs/msg/vs_graphs_all_detectdet_rooms.hpp>

using json = nlohmann::json;

class ORB_SLAM3::SystemParams;
extern ORB_SLAM3::System *pSLAM;
extern ORB_SLAM3::System::eSensor sensorType;

extern bool colorPointcloud;
extern double roll, pitch, yaw;
extern bool pubStaticTransform, pubPointClouds;
extern std::string frameCamera, frameImu, frameWorld, frameMap, frameBC, frameSE;

// TF broadcasters
extern std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
extern std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;

// List of visited Fiducial Markers in different timestamps
extern std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;

// List of white space cluster points obtained from `voxblox_skeleton`
extern std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

// List of GNN-based room candidates
extern std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;

// List of wall publishers for GNN-based room detection variants
extern rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr pubAllWalls_new;
extern rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr pubAllWalls_legacy;

extern double lastPlanePublishTime;
extern std::shared_ptr<image_transport::Publisher> pubTrackingImage;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
extern rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr pubKFImage;
extern rclcpp::Publisher<keyframe_depth_estimator::msg::KeyFrameCreated>::SharedPtr pubKeyFrameCreated;
extern rclcpp::Publisher<keyframe_depth_validator::msg::StaticMapPointCorrespondences>::SharedPtr pubKeyFrameStaticMapPoints;
extern rclcpp::Publisher<keyframe_depth_validator::msg::StaticMapPoints>::SharedPtr pubStaticMapPoints;
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCameraPose;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllMappoints;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrackedMappoints;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentedPointcloud;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubWorldFramePointCloud;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubCameraPoseVis;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKeyFrameMarker;
extern rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubStructuralElements;

class MapPointStruct
{
    int clusterId;
    Eigen::Vector3f coordinates;
    MapPointStruct(Eigen::Vector3f coords) : coordinates(coords), clusterId(-1) {}
};

void setupServices(std::shared_ptr<rclcpp::Node>, const std::string &);
void publishFramePointCloud(Sophus::SE3f, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL, rclcpp::Time);
void publishTopics(rclcpp::Time, Eigen::Vector3f = Eigen::Vector3f::Zero(), const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL = nullptr);
void setupPublishers(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<image_transport::ImageTransport> image_transport, const std::string &node_name);

void publishTrackingImage(cv::Mat, rclcpp::Time);
void publishDoors(std::vector<ORB_SLAM3::Door *>);
void publishCameraPose(Sophus::SE3f, rclcpp::Time);
void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *>);
void publishPlanes(std::vector<ORB_SLAM3::Plane *>, rclcpp::Time);
void publishTFTransform(Sophus::SE3f, string, string, rclcpp::Time);
void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);
void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);
void publishStaticMapPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);
void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *>, rclcpp::Time);
void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);
void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);
void publishBodyOdometry(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, rclcpp::Time);
void publishStructuralElements(std::vector<ORB_SLAM3::Room *>, std::vector<ORB_SLAM3::Floor *>, rclcpp::Time);

/**
 * @brief Publishes all mapped walls to detect possible rooms (mainly used in GNN-based room detector).
 *
 * @param walls The vector of mapped walls to be published.
 * @param time The timestamp for the message.
 */
void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *>, rclcpp::Time);

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *>);

void saveMapService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> request,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> response);

void saveTrajectoryService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> request,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> response);

void saveMapPointsAsPCDService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> request,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> response);

/**
 * @brief Converts a SE3f to a cv::Mat
 *
 * @param data The SE3f data to be converted
 */
cv::Mat SE3fToCvMat(Sophus::SE3f data);

/**
 * @brief Converts a SE3f to a tf::Transform
 *
 * @param data The SE3f data to be converted
 */
tf2::Transform SE3fToTFTransform(Sophus::SE3f data);

/**
 * @brief Converts a vector of MapPoints to a PointCloud2 message
 *
 * @param mapPoints The vector of MapPoints to be converted
 * @param msgTime The timestamp for the PointCloud2 message
 */
sensor_msgs::msg::PointCloud2 mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints, rclcpp::Time msgTime);

/**
 * @brief Publishes a static transformation (TF) between two coordinate frames and define a
 * fixed spatial relationship among them.
 *
 * @param parentFrameId The parent frame ID for the static transformation
 * @param childFrameId The child frame ID for the static transformation
 * @param msgTime The timestamp for the transformation message
 */
void publishStaticTFTransform(string parentFrameId, string childFrameId, rclcpp::Time msgTime);

/**
 * @brief Publishes the free space clusters obtained from `voxblox_skeleton` as a PointCloud2 message
 *
 * @param skeletonClusterPoints The list of free space cluster points
 * @param msgTime The timestamp for the PointCloud2 message
 */
void publishFreeSpaceClusters(std::vector<std::vector<Eigen::Vector3d>>, rclcpp::Time);

/**
 * @brief Adds the markers to the buffer to be processed
 * @param markerArray The array of markers received from `aruco_ros`
 */
// void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray);

/**
 * @brief Avoids adding duplicate markers to the buffer by checking the timestamp
 * @param frameTimestamp The timestamp of the frame that captured the marker
 */
std::pair<double, std::vector<ORB_SLAM3::Marker *>> findNearestMarker(double frameTimestamp);

/**
 * @brief Gets skeleton voxels from `voxblox_skeleton` to be processed
 * @param skeletonArray The array of skeleton voxels received
 */
void setVoxbloxSkeletonCluster(const visualization_msgs::msg::MarkerArray &skeletonArray);

/**
 * @brief Gets the set of room candidates detected by the GNN-based room detection module
 * Mainly designed for the legacy version of the GNN-based room detector
 * @param msgGNNRooms The message containing the detected room candidates
 */
void setGNNBasedRoomCandidates(const situational_graphs_msgs::msg::RoomsData &msgGNNRooms);

/**
 * @brief Gets the set of room candidates detected by the GNN-based room detection module
 * Mainly designed for the new version of the GNN-based room detector
 * @param msgGNNRooms The message containing the detected room candidates
 */
void setGNNBasedRoomCandidates(const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms);
