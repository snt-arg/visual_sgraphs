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

#include "common.h"
#include <rclcpp/rclcpp.hpp>

// Variables for ORB-SLAM3
ORB_SLAM3::System *pSLAM;
ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
bool colorPointcloud = true;
double roll = 0, pitch = 0, yaw = 0;
bool pubStaticTransform, pubPointClouds;
std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;
std::shared_ptr<image_transport::Publisher> pubTrackingImage;
// std::shared_ptr<tf2::TransformListener> transformListener;
std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;
std::string frameWorld, frameCamera, frameImu, frameMap, frameBC, frameSE;

rclcpp::Time lastPlanePublishTime(0, 0, RCL_ROS_TIME);
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubKeyFrameList;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllMappoints;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCameraPose;
rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr pubKFImage;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPassage;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBuildingComponents;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrackedMappoints;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreespaceCluster;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPlaneLabel;
rclcpp::Publisher<vs_graphs::msg::VSGraphsAllWallsData>::SharedPtr pubAllWalls_new;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentedPointcloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubWorldFramePointCloud;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubCameraPoseVis;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKeyFrameMarker;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubFiducialMarker;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubStructuralElements;
rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr pubAllWalls_legacy;

void saveMapService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    res->success = pSLAM->SaveMap(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"), "Map was saved as %s.osa", req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Map could not be saved.");
}

void saveMapPointsAsPCDService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    res->success = pSLAM->SaveMapPointsAsPCD(req->name);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"), "Map points were saved as %s.pcd", req->name.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Map points could not be saved.");
}

void saveTrajectoryService(
    std::shared_ptr<vs_graphs::srv::SaveMap::Request> req,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response> res)
{
    const std::string cameraTrajectoryFile = req->name + "_cam_traj.txt";
    const std::string keyframeTrajectoryFile = req->name + "_kf_traj.txt";

    try
    {
        pSLAM->SaveTrajectoryEuRoC(cameraTrajectoryFile);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(keyframeTrajectoryFile);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        res->success = false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
        res->success = false;
    }

    if (!res->success)
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Estimated trajectory could not be saved.");
}

void setupServices(std::shared_ptr<rclcpp::Node> node, const std::string &node_name)
{
    node->create_service<vs_graphs::srv::SaveMap>(
        node_name + "/save_map", &saveMapService);
    node->create_service<vs_graphs::srv::SaveMap>(
        node_name + "/save_map_points", &saveMapPointsAsPCDService);
    node->create_service<vs_graphs::srv::SaveMap>(
        node_name + "/save_traj", &saveTrajectoryService);
}

void setupPublishers(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<image_transport::ImageTransport> image_transport, const std::string &node_name)
{
    // Basic
    pubKeyFrameList = node->create_publisher<nav_msgs::msg::Path>(node_name + "/keyframe_list", 2);
    pubAllMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", 1);
    pubCameraPose = node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", 1);
    pubKFImage = node->create_publisher<segmenter_ros::msg::VSGraphDataMsg>(node_name + "/keyframe_image", 50);
    pubTrackedMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", 1);
    pubWorldFramePointCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/points_map", 1);
    pubKeyFrameMarker = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/kf_markers", 1);
    pubFreespaceCluster = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/freespace_clusters", 1);
    pubCameraPoseVis = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/camera_pose_vis", 1);
    pubTrackingImage = std::make_shared<image_transport::Publisher>(image_transport->advertise(node_name + "/tracking_image", 1));

    // Entities
    pubPassage = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/passages", 1);
    pubFiducialMarker = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/fiducial_markers", 1);

    // Building Components
    pubPlaneLabel = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/plane_labels", 1);
    pubBuildingComponents = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/building_components", 1);
    pubSegmentedPointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/segmented_point_clouds", 1);

    // All Walls
    pubAllWalls_new = node->create_publisher<vs_graphs::msg::VSGraphsAllWallsData>(node_name + "/all_mapped_walls", 1);
    // pubAllWalls_legacy = node->create_publisher<situational_graphs_msgs::msg::PlanesData>(node_name + "/all_mapped_walls", 1);

    // Structural Elements
    pubStructuralElements = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/structural_elements", 1);

    // Get body odometry if IMU data is also available
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR || sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
        pubOdometry = node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", 1);

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void publishTopics(rclcpp::Time msgTime, Eigen::Vector3f Wbb, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    // Avoid publishing NaN
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0))
        return;

    // Common topics
    publishCameraPose(Twc, msgTime);
    publishTFTransform(Twc, frameWorld, frameCamera, msgTime);
    publishFramePointCloud(Twc, msgPCL, msgTime);

    // Set a static transform between the world and map frame
    if (pubStaticTransform)
        publishStaticTFTransform(frameWorld, frameMap, msgTime);

    // Get KeyFrames
    std::vector<ORB_SLAM3::KeyFrame *> keyframes = pSLAM->GetAllKeyFrames();

    // Setup publishers
    publishKeyFrameImages(keyframes, msgTime);
    publishKeyFrameMarkers(keyframes, msgTime);
    publishPassages(pSLAM->GetAllPassages(), msgTime);
    publishFiducialMarkers(pSLAM->GetAllMarkers(), msgTime);
    publishTrackingImage(pSLAM->GetCurrentFrame(), msgTime);
    publishStructuralElements(pSLAM->GetAllRooms(), pSLAM->GetAllFloors(), msgTime);

    // Publish all mapped walls for GNN-based room detection
    publishAllMappedWalls(pSLAM->GetAllPlanes(), msgTime);

    // Publish pointclouds
    if (pubPointClouds)
    {
        publishSegmentedCloud(keyframes);
        publishPlanes(pSLAM->GetAllPlanes(), msgTime);
        publishAllPoints(pSLAM->GetAllMapPoints(), msgTime);
        publishTrackedPoints(pSLAM->GetTrackedMapPoints(), msgTime);
        publishFreeSpaceClusters(pSLAM->getSkeletonCluster(), msgTime);
    }
    else
        clearKFClsClouds(keyframes);

    // IMU-specific topics
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR || sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
    {
        // Body pose and translational velocity can be obtained from ORB-SLAM3
        Sophus::SE3f Twb = pSLAM->GetImuTwb();
        Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

        // IMU provides body angular velocity in body frame (Wbb) which is transformed to world frame (Wwb)
        Sophus::Matrix3f Rwb = Twb.rotationMatrix();
        Eigen::Vector3f Wwb = Rwb * Wbb;

        publishTFTransform(Twb, frameWorld, frameImu, msgTime);
        publishBodyOdometry(Twb, Vwb, Wwb, msgTime);
    }
}

void publishBodyOdometry(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msgTime)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.child_frame_id = frameImu;
    odom_msg.header.frame_id = frameWorld;
    odom_msg.header.stamp = msgTime;

    odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

    odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
    odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
    odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
    odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

    odom_msg.twist.twist.linear.x = Vwb_E3f.x();
    odom_msg.twist.twist.linear.y = Vwb_E3f.y();
    odom_msg.twist.twist.linear.z = Vwb_E3f.z();

    odom_msg.twist.twist.angular.x = ang_vel_body.x();
    odom_msg.twist.twist.angular.y = ang_vel_body.y();
    odom_msg.twist.twist.angular.z = ang_vel_body.z();

    pubOdometry->publish(odom_msg);
}

void publishCameraPose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msgTime)
{
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.frame_id = frameCamera;
    poseMsg.header.stamp = msgTime;

    poseMsg.pose.position.x = Tcw_SE3f.translation().x();
    poseMsg.pose.position.y = Tcw_SE3f.translation().y();
    poseMsg.pose.position.z = Tcw_SE3f.translation().z();

    poseMsg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    poseMsg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    poseMsg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    poseMsg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pubCameraPose->publish(poseMsg);

    // Add a marker for visualization
    visualization_msgs::msg::Marker cameraVisual;
    visualization_msgs::msg::MarkerArray cameraVisualList;

    cameraVisual.id = 1;
    cameraVisual.color.a = 0.7;
    cameraVisual.scale.x = 0.5;
    cameraVisual.scale.y = 0.5;
    cameraVisual.scale.z = 0.5;
    cameraVisual.ns = "camera_pose";
    cameraVisual.header.stamp = msgTime;
    cameraVisual.action = cameraVisual.ADD;
    cameraVisual.header.frame_id = frameWorld;
    cameraVisual.mesh_use_embedded_materials = true;
    cameraVisual.lifetime = rclcpp::Duration::from_seconds(0);
    cameraVisual.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    cameraVisual.mesh_resource =
        "package://vs_graphs/config/Assets/camera.dae";

    cameraVisual.pose.position.x = Tcw_SE3f.translation().x();
    cameraVisual.pose.position.y = Tcw_SE3f.translation().y();
    cameraVisual.pose.position.z = Tcw_SE3f.translation().z();
    cameraVisual.pose.orientation.x = Tcw_SE3f.unit_quaternion().x();
    cameraVisual.pose.orientation.y = Tcw_SE3f.unit_quaternion().y();
    cameraVisual.pose.orientation.z = Tcw_SE3f.unit_quaternion().z();
    cameraVisual.pose.orientation.w = Tcw_SE3f.unit_quaternion().w();

    cameraVisualList.markers.push_back(cameraVisual);

    pubCameraPoseVis->publish(cameraVisualList);
}

void publishTFTransform(Sophus::SE3f T_SE3f, std::string parentFrameId, std::string childFrameId, rclcpp::Time msgTime)
{
    // Variables
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = msgTime;
    transformStamped.child_frame_id = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the values of transform messages
    transformStamped.transform.translation.x = T_SE3f.translation().x();
    transformStamped.transform.translation.y = T_SE3f.translation().y();
    transformStamped.transform.translation.z = T_SE3f.translation().z();

    // Fill rotation
    transformStamped.transform.rotation.x = T_SE3f.unit_quaternion().x();
    transformStamped.transform.rotation.y = T_SE3f.unit_quaternion().y();
    transformStamped.transform.rotation.z = T_SE3f.unit_quaternion().z();
    transformStamped.transform.rotation.w = T_SE3f.unit_quaternion().w();

    if (tfBroadcaster)
        tfBroadcaster->sendTransform(transformStamped);
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "TF broadcaster is not initialized.");
}

void publishStaticTFTransform(std::string parentFrameId, std::string childFrameId, rclcpp::Time msgTime)
{
    // Variables
    tf2::Quaternion quat;
    geometry_msgs::msg::TransformStamped transformStamped;

    // Set the values of transform messages
    transformStamped.header.stamp = msgTime;
    transformStamped.child_frame_id = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the translation to zero (static transform)
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    // Set the rotation using roll, pitch, yaw
    quat.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    if (staticTfBroadcaster)
        staticTfBroadcaster->sendTransform(transformStamped);
    else
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), "Static TF broadcaster is not initialized.");
}

void publishFreeSpaceClusters(std::vector<std::vector<Eigen::Vector3d>> clusterPoints, rclcpp::Time msgTime)
{
    // Check if the cluster points are empty
    if (clusterPoints.empty())
        return;

    // Variables
    int colorIndex = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr freeSpaceCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<std::vector<uint8_t>> fixedColors = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}, {128, 0, 0}};

    // Loop through all the cluster points and add them to the point cloud
    for (const auto &cluster : clusterPoints)
    {
        // Variables
        std::vector<uint8_t> color = fixedColors[colorIndex];
        // Loop through all the points in the current cluster
        for (const auto &point : cluster)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x();
            newPoint.y = point.y();
            newPoint.z = point.z();
            newPoint.r = color[0];
            newPoint.g = color[1];
            newPoint.b = color[2];
            freeSpaceCloud->push_back(newPoint);
        }
        // Increment the color index
        colorIndex += 1;
        // if (colorIndex == fixedColors.size())
        if (colorIndex == static_cast<int>(fixedColors.size()))
            colorIndex = 0;
    }

    // Check if the point cloud is empty
    if (freeSpaceCloud->empty())
        return;

    // Convert the point cloud to a PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*freeSpaceCloud, cloudMsg);

    // Set message header
    cloudMsg.header.stamp = msgTime;
    cloudMsg.header.frame_id = frameWorld;

    // Publish the point cloud
    pubFreespaceCluster->publish(cloudMsg);
}

void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, rclcpp::Time msgTime)
{
    // Check all keyframes and publish the ones that have not been published for Semantic Segmentation yet
    for (auto &keyframe : keyframe_vec)
    {
        if (keyframe->isPublished)
            continue;

        // Create an object of VSGraphDataMsg
        segmenter_ros::msg::VSGraphDataMsg vsGraphPublisher = segmenter_ros::msg::VSGraphDataMsg();
        std_msgs::msg::Header header;
        header.stamp = msgTime;
        header.frame_id = frameWorld;
        std_msgs::msg::UInt64 kfId;
        kfId.data = keyframe->mnId;
        const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
            cv_bridge::CvImage(header, "bgr8", keyframe->mImage).toImageMsg();

        vsGraphPublisher.header = header;
        vsGraphPublisher.key_frame_id = kfId;
        vsGraphPublisher.key_frame_image = *rendered_image_msg;

        pubKFImage->publish(vsGraphPublisher);
        keyframe->isPublished = true;
    }
}

void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *> walls, rclcpp::Time msgTime)
{
    // Check the proper version of the GNN-based room detection
    bool isLegacy = ORB_SLAM3::SystemParams::GetParams()->room_seg.gnn_version == 1;

    if (isLegacy)
    {
        // Variables
        // situational_graphs_msgs::msg::WallsData wallDataMsg;

        // // Fill the data message with wall information
        // wallDataMsg.header.stamp = msgTime;
        // wallDataMsg.header.frame_id = frameWorld;

        // // Fill in the walls data
        // for (const auto &wall : walls)
        // {
        //     if (!wall || wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
        //         continue;

        //     // Fill the wall data
        //     situational_graphs_msgs::msg::PlaneData wallData;

        //     // wallData.length = length;
        //     wallData.id = wall->getId();
        //     // wallData.d = wall->getGlobalEquation().d();
        //     wallData.nx = wall->getGlobalEquation().normal().x();
        //     wallData.ny = wall->getGlobalEquation().normal().y();
        //     wallData.nz = wall->getGlobalEquation().normal().z();

        //     // Add the wall to the message
        //     wallDataMsg.walls.push_back(wallData);
        // }

        // // Publish all mapped walls
        // pubAllWalls_legacy->publish(wallDataMsg);
    }
    else
    {
        // Variables
        vs_graphs::msg::VSGraphsAllWallsData wallDataMsg;

        // Fill the data message with wall information
        wallDataMsg.header.stamp = msgTime;
        wallDataMsg.header.frame_id = frameWorld;

        // Fill in the walls data
        for (const auto &wall : walls)
        {
            if (!wall || wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
                continue;

            // Calculate the length of the wall
            float length = 0.0f;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud = wall->getMapClouds();
            if (wallCloud && wallCloud->points.size() > 1)
            {
                // Calculate the length of the wall by finding the distance between the first and last points
                Eigen::Vector3f startPoint(wallCloud->points.front().x, wallCloud->points.front().y, wallCloud->points.front().z);
                Eigen::Vector3f endPoint(wallCloud->points.back().x, wallCloud->points.back().y, wallCloud->points.back().z);
                length = (endPoint - startPoint).norm();
            }

            // Fill the wall data
            vs_graphs::msg::VSGraphsWallData wallData;

            wallData.length = length;
            wallData.id = wall->getId();
            wallData.centroid.x = wall->getCentroid().x();
            wallData.centroid.y = wall->getCentroid().y();
            wallData.centroid.z = wall->getCentroid().z();
            wallData.normal.x = wall->getGlobalEquation().normal().x();
            wallData.normal.y = wall->getGlobalEquation().normal().y();
            wallData.normal.z = wall->getGlobalEquation().normal().z();

            // Add the wall to the message
            wallDataMsg.walls.push_back(wallData);
        }

        // Publish all mapped walls
        pubAllWalls_new->publish(wallDataMsg);
    }
}

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec)
{
    for (auto &keyframe : keyframe_vec)
        keyframe->clearClsClouds();
}

void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec)
{
    // get the latest processed keyframe
    ORB_SLAM3::KeyFrame *thisKF = nullptr;
    for (int i = keyframe_vec.size() - 1; i >= 0; i--)
    {
        if (keyframe_vec[i]->getClsCloudPtrs().size() > 0)
        {
            thisKF = keyframe_vec[i];
            // clear all clsClouds from the keyframes prior to the index i
            for (int j = 0; j < i; j++)
                keyframe_vec[j]->clearClsClouds();
            break;
        }
    }
    if (thisKF == nullptr)
        return;

    // get the class specific pointclouds from this keyframe
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs = thisKF->getClsCloudPtrs();

    // create a new pointcloud with aggregated points from all classes but with class-specific colors
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (long unsigned int i = 0; i < clsCloudPtrs.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clsCloud = clsCloudPtrs[i];
        for (long unsigned int j = 0; j < clsCloud->points.size(); j++)
        {
            pcl::PointXYZRGBA point = clsCloud->points[j];
            switch (i)
            {
            case 0: // Ground is green
                point.r = 0;
                point.g = 255;
                point.b = 0;
                break;
            case 1: // Wall is red
                point.r = 255;
                point.g = 0;
                point.b = 0;
                break;
            }
            aggregatedCloud->push_back(point);
        }
    }
    aggregatedCloud->header = clsCloudPtrs[0]->header;
    thisKF->clearClsClouds();

    // create a new pointcloud2 message from the transformed and aggregated pointcloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregatedCloud, cloud_msg);

    // publish the pointcloud to be seen at the plane frame
    cloud_msg.header.frame_id = frameCamera;
    pubSegmentedPointcloud->publish(cloud_msg);
}

void publishTrackingImage(cv::Mat image, rclcpp::Time msgTime)
{
    std_msgs::msg::Header header;
    header.stamp = msgTime;
    header.frame_id = frameWorld;
    const sensor_msgs::msg::Image::SharedPtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pubTrackingImage->publish(rendered_image_msg);
}

void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *> trackedMapPoints, rclcpp::Time msgTime)
{
    sensor_msgs::msg::PointCloud2 cloud = mapPointToPointcloud(trackedMapPoints, msgTime);
    pubTrackedMappoints->publish(cloud);
}

void publishFramePointCloud(Sophus::SE3f Twc, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL, rclcpp::Time msgTime)
{
    if (!msgPCL)
        return;

    // Transform the point cloud to the world frame
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*msgPCL, *cloud);

    // Transform the point cloud to the world frame
    Eigen::Matrix4f Twc_eigen = Twc.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud, *transformedCloud, Twc_eigen);

    pcl::toROSMsg(*transformedCloud, cloudMsg);

    cloudMsg.header.stamp = msgTime;
    cloudMsg.header.frame_id = frameWorld;

    pubWorldFramePointCloud->publish(cloudMsg);
}

void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> allMapPoints, rclcpp::Time msgTime)
{
    sensor_msgs::msg::PointCloud2 cloud = mapPointToPointcloud(allMapPoints, msgTime);
    pubAllMappoints->publish(cloud);
}

void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, rclcpp::Time msgTime)
{
    sort(keyframe_vec.begin(), keyframe_vec.end(), ORB_SLAM3::KeyFrame::lId);
    if (keyframe_vec.size() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = frameWorld;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration::from_seconds(0);
    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    visualization_msgs::msg::Marker kf_lines;
    kf_lines.id = 1;
    kf_lines.color.a = 0.15;
    kf_lines.color.r = 0.0;
    kf_lines.color.g = 0.0;
    kf_lines.color.b = 0.0;
    kf_lines.scale.x = 0.003;
    kf_lines.scale.y = 0.003;
    kf_lines.scale.z = 0.003;
    kf_lines.action = kf_lines.ADD;
    kf_lines.ns = "kf_lines";
    kf_lines.lifetime = rclcpp::Duration::from_seconds(0);
    kf_lines.header.stamp = rclcpp::Clock().now();
    kf_lines.header.frame_id = frameWorld;
    kf_lines.type = visualization_msgs::msg::Marker::LINE_LIST;

    nav_msgs::msg::Path kf_list;
    kf_list.header.frame_id = frameWorld;
    kf_list.header.stamp = msgTime;

    for (auto &keyframe : keyframe_vec)
    {
        geometry_msgs::msg::Point kf_marker;

        Sophus::SE3f kf_pose = pSLAM->GetKeyFramePose(keyframe);
        kf_marker.x = kf_pose.translation().x();
        kf_marker.y = kf_pose.translation().y();
        kf_marker.z = kf_pose.translation().z();
        kf_markers.points.push_back(kf_marker);

        // Populate the keyframe list
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frameWorld;
        pose.pose.position.x = kf_pose.translation().x();
        pose.pose.position.y = kf_pose.translation().y();
        pose.pose.position.z = kf_pose.translation().z();
        pose.pose.orientation.w = kf_pose.unit_quaternion().w();
        pose.pose.orientation.x = kf_pose.unit_quaternion().x();
        pose.pose.orientation.y = kf_pose.unit_quaternion().y();
        pose.pose.orientation.z = kf_pose.unit_quaternion().z();
        pose.header.stamp = rclcpp::Time(keyframe->mTimeStamp * 1e9);
        kf_list.poses.push_back(pose);
    }

    markerArray.markers.push_back(kf_markers);
    pubKeyFrameMarker->publish(markerArray);
    pubKeyFrameList->publish(kf_list);
}

void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *> markers, rclcpp::Time msgTime)
{
    int numMarkers = markers.size();
    if (numMarkers == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.resize(numMarkers);

    for (int idx = 0; idx < numMarkers; idx++)
    {
        visualization_msgs::msg::Marker fiducial_marker;
        Sophus::SE3f markerPose = markers[idx]->getGlobalPose();

        fiducial_marker.color.a = 0;
        fiducial_marker.scale.x = 0.2;
        fiducial_marker.scale.y = 0.2;
        fiducial_marker.scale.z = 0.2;
        fiducial_marker.ns = "fiducial_markers";
        fiducial_marker.lifetime = rclcpp::Duration::from_seconds(0);
        fiducial_marker.action = fiducial_marker.ADD;
        fiducial_marker.id = markerArray.markers.size();
        fiducial_marker.header.stamp = rclcpp::Clock().now(); // rclcpp::Time().now();
        fiducial_marker.mesh_use_embedded_materials = true;
        fiducial_marker.header.frame_id = frameBC;
        fiducial_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        fiducial_marker.mesh_resource =
            "package://vs_graphs/config/Assets/aruco_marker.dae";

        fiducial_marker.pose.position.x = markerPose.translation().x();
        fiducial_marker.pose.position.y = markerPose.translation().y();
        fiducial_marker.pose.position.z = markerPose.translation().z();
        fiducial_marker.pose.orientation.x = markerPose.unit_quaternion().x();
        fiducial_marker.pose.orientation.y = markerPose.unit_quaternion().y();
        fiducial_marker.pose.orientation.z = markerPose.unit_quaternion().z();
        fiducial_marker.pose.orientation.w = markerPose.unit_quaternion().w();

        markerArray.markers.push_back(fiducial_marker);
    }

    pubFiducialMarker->publish(markerArray);
}

void publishPassages(std::vector<ORB_SLAM3::Passage *> passages, rclcpp::Time msgTime)
{
    // If there are no passages, return
    int numPassages = passages.size();
    if (numPassages == 0)
        return;

    // Variables
    visualization_msgs::msg::MarkerArray passageArray;
    passageArray.markers.resize(numPassages);

    for (int idx = 0; idx < numPassages; idx++)
    {
        // Variables
        Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
        visualization_msgs::msg::Marker passage;

        // Get the planar parameters of the passage
        double width = passages[idx]->getWidth();
        double height = passages[idx]->getHeight();
        bool isPassable = passages[idx]->isPassable();
        Eigen::Vector3f centroid = passages[idx]->getCentroid();
        Eigen::Vector3d normal = passages[idx]->getGlobalEquation().normal().normalized();
        bool isDoorway = passages[idx]->getPassageType() == ORB_SLAM3::Passage::passageVariant::DOORWAY;

        // Orientation of the passage marker
        if (normal.dot(z_axis) < 0)
            normal = -normal;
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(z_axis, normal);
        quat.normalize();

        // Passage values
        passage.id = idx;
        passage.color.a = 0.5;
        passage.ns = "passage";
        passage.action = passage.ADD;
        passage.header.stamp = msgTime;
        passage.header.frame_id = frameBC;
        passage.mesh_use_embedded_materials = true;
        passage.lifetime = rclcpp::Duration::from_seconds(0);
        passage.type = visualization_msgs::msg::Marker::CUBE;
        // passage.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        // passage.mesh_resource = "package://vs_graphs/config/Assets/doorframe.obj";

        // Set color (closed passages are red, open passages are blue, doorways are green)
        std::vector<double> color = {1.0, 0.0, 0.0};
        if (isPassable)
            color = isDoorway ? std::vector<double>{0.5, 1.0, 0.5} : std::vector<double>{0.5, 0.5, 1.0};
        passage.color.r = color[0];
        passage.color.g = color[1];
        passage.color.b = color[2];

        // Position: always from centroid
        passage.pose.position.x = static_cast<double>(centroid.x());
        passage.pose.position.y = static_cast<double>(centroid.y());
        passage.pose.position.z = static_cast<double>(centroid.z());

        // Orientation: always from plane normal
        passage.pose.orientation.x = quat.x();
        passage.pose.orientation.y = quat.y();
        passage.pose.orientation.z = quat.z();
        passage.pose.orientation.w = quat.w();

        // Set shape and size of the passage marker
        passage.scale.z = 0.1;
        passage.scale.y = width;
        passage.scale.x = height;
        // passage.scale.z = 1.0;
        // passage.scale.y = width / ORB_SLAM3::SystemParams::GetParams()->sem_seg.max_door_width;
        // passage.scale.x = height / ORB_SLAM3::SystemParams::GetParams()->sem_seg.max_door_height;

        passageArray.markers.push_back(passage);
    }

    pubPassage->publish(passageArray);
}

void publishPlanes(std::vector<ORB_SLAM3::Plane *> planes, rclcpp::Time msgTime)
{
    // Publish the planes, if any
    int numPlanes = planes.size();
    if (numPlanes == 0)
        return;

    // Check if sufficient time has passed since the last plane publication
    if ((msgTime - lastPlanePublishTime).seconds() < 3.0)
        return;

    lastPlanePublishTime = msgTime;

    // Variables
    visualization_msgs::msg::MarkerArray planeLabelArray;
    planeLabelArray.markers.resize(numPlanes);
    visualization_msgs::msg::Marker planeLabel, planeNormal;
    geometry_msgs::msg::Point normalStartPoint, normalEndPoint;

    // Aggregate pointcloud XYZRGB for all planes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Loop through all the planes
    for (const auto &plane : planes)
    {
        // Variables
        std::vector<uint8_t> color = plane->getColor();
        Eigen::Vector3f centroid = plane->getCentroid();
        Eigen::Vector3d normal = plane->getGlobalEquation().normal();
        const std::string planeLabelText = "Plane#" + std::to_string(plane->getId());

        // If the plane is undefined, skip it
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
            continue;

        // Get the point clouds for the plane
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeClouds = plane->getMapClouds();
        if (planeClouds == nullptr || planeClouds->empty())
            continue;

        for (const auto &point : planeClouds->points)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x;
            newPoint.y = point.y;
            newPoint.z = point.z;
            newPoint.r = point.r;
            newPoint.g = point.g;
            newPoint.b = point.b;

            // [TEMP] Override color according to type of plane
            // if (colorPointcloud)
            // {
            //     newPoint.r = color[0];
            //     newPoint.g = color[1];
            //     newPoint.b = color[2];
            // }

            // If the plane is a door, visualize it in a distinct color (purple)
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::DOOR)
            {
                newPoint.r = 204;
                newPoint.g = 0;
                newPoint.b = 102;
            }

            // Add the point to the aggregated cloud
            aggregatedCloud->push_back(newPoint);
        }

        // Print the plane ID on the center of the plane
        planeLabel.color.a = 1.0;
        planeLabel.scale.z = 0.2;
        planeLabel.ns = "planeLabel";
        planeLabel.id = plane->getId();
        planeLabel.text = planeLabelText;
        planeLabel.header.stamp = msgTime;
        planeLabel.action = planeLabel.ADD;
        planeLabel.header.frame_id = frameBC;
        planeLabel.color.r = color[0] / 255.0;
        planeLabel.color.g = color[1] / 255.0;
        planeLabel.color.b = color[2] / 255.0;
        planeLabel.pose.position.x = centroid.x();
        planeLabel.pose.position.z = centroid.z();
        planeLabel.pose.position.y = centroid.y() - 1.5;
        planeLabel.lifetime = rclcpp::Duration::from_seconds(0);
        planeLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        // Create a marker for the plane normal (as an arrow)
        planeNormal.color.a = 1.0;
        planeNormal.scale.x = 0.01; // Shaft diameter
        planeNormal.scale.y = 0.05; // Arrowhead diameter
        planeNormal.scale.z = 0.05; // Arrowhead length
        planeNormal.ns = "planeNormal";
        planeNormal.header.stamp = msgTime;
        planeNormal.header.frame_id = frameBC;
        planeNormal.color.r = color[0] / 255.0;
        planeNormal.color.g = color[1] / 255.0;
        planeNormal.color.b = color[2] / 255.0;
        planeNormal.id = plane->getId() + numPlanes;
        planeNormal.lifetime = rclcpp::Duration::from_seconds(0);
        planeNormal.type = visualization_msgs::msg::Marker::ARROW;

        // Clear previous points from planeNormal
        planeNormal.points.clear();

        // Set the arrow's start and end points
        normalStartPoint.x = centroid.x();
        normalStartPoint.y = centroid.y();
        normalStartPoint.z = centroid.z();
        normalEndPoint.x = normalStartPoint.x + normal.x() * 0.2;
        normalEndPoint.y = normalStartPoint.y + normal.y() * 0.2;
        normalEndPoint.z = normalStartPoint.z + normal.z() * 0.2;

        planeNormal.points.push_back(normalStartPoint);
        planeNormal.points.push_back(normalEndPoint);

        // Add the normal marker to the marker array
        planeLabelArray.markers.push_back(planeLabel);
        planeLabelArray.markers.push_back(planeNormal);
    }

    if (aggregatedCloud->empty())
        return;

    // Convert the aggregated pointcloud to a pointcloud2 message
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*aggregatedCloud, cloudMsg);

    // Set message header
    cloudMsg.header.stamp = msgTime;
    cloudMsg.header.frame_id = frameBC;

    // Publish the point cloud
    pubBuildingComponents->publish(cloudMsg);
    pubPlaneLabel->publish(planeLabelArray);
}

void publishStructuralElements(std::vector<ORB_SLAM3::Room *> rooms,
                               std::vector<ORB_SLAM3::Floor *> floors, rclcpp::Time msgTime)
{
    // Publish rooms, if any
    int numRooms = rooms.size();
    int numFloors = floors.size();

    // If there are no rooms or floors, return
    if (numRooms == 0 && numFloors == 0)
        return;

    // Variables
    double textOffset = -0.5;
    double floorToRoomOffset = -2.0;

    // Visualization markers
    visualization_msgs::msg::MarkerArray roomArray, floorArray;
    roomArray.markers.resize(numRooms);
    floorArray.markers.resize(numFloors);

    // Publish rooms, if any
    if (numRooms > 0)
    {
        for (int idx = 0; idx < numRooms; idx++)
        {
            // Skip if the room is bad
            if (rooms[idx]->isBad())
            {
                // Variables
                visualization_msgs::msg::Marker delRoom, delRoomLabel, delRoomWallLine;
                // Delete previous marker for this room
                delRoom.id = idx;
                delRoom.ns = "room";
                delRoom.header.stamp = msgTime;
                delRoom.header.frame_id = frameSE;
                delRoom.action = visualization_msgs::msg::Marker::DELETE;
                // Delete previous marker for this room label
                delRoomLabel.id = idx;
                delRoomLabel.ns = "roomLabel";
                delRoomLabel.header.stamp = msgTime;
                delRoomLabel.header.frame_id = frameSE;
                delRoomLabel.action = visualization_msgs::msg::Marker::DELETE;
                // Delete previous room-wall lines
                delRoomWallLine.id = idx;
                delRoomWallLine.ns = "roomWallLine";
                delRoomWallLine.header.stamp = msgTime;
                delRoomWallLine.header.frame_id = frameWorld;
                delRoomWallLine.action = visualization_msgs::msg::Marker::DELETE;
                // Push the delete markers and skip them
                roomArray.markers.push_back(delRoom);
                roomArray.markers.push_back(delRoomLabel);
                roomArray.markers.push_back(delRoomWallLine);
                continue;
            }

            // Variables
            std::string roomName = rooms[idx]->getName();
            geometry_msgs::msg::PointStamped roomPoint, roomPointTr;

            // Create color based on room type (undefined: gray, corridor: dark pink, room: purple)
            std::vector<double> color = {0.5, 0.5, 0.5};
            if (rooms[idx]->getRoomVariant() == ORB_SLAM3::Room::roomVariant::CORRIDOR)
                color = {0.6, 0.0, 0.3};
            if (rooms[idx]->getRoomVariant() == ORB_SLAM3::Room::roomVariant::ROOM)
                color = {0.5, 0.1, 1.0};

            Eigen::Vector3d centroid = rooms[idx]->getCentroid();
            visualization_msgs::msg::Marker room, roomWallLine, roomDoorwayLine, roomMarkerLine, roomLabel;

            // Room values
            room.id = idx;
            room.ns = "room";
            room.scale.x = 0.3;
            room.scale.y = 0.3;
            room.scale.z = 0.3;
            room.color.a = 1.0;
            room.action = room.ADD;
            room.color.r = color[0];
            room.color.g = color[1];
            room.color.b = color[2];
            room.header.stamp = msgTime;
            room.pose.orientation.x = 0.0;
            room.pose.orientation.y = 0.0;
            room.pose.orientation.z = 0.0;
            room.pose.orientation.w = 1.0;
            room.header.frame_id = frameSE;
            room.pose.position.x = centroid.x();
            room.pose.position.y = centroid.y();
            room.pose.position.z = centroid.z();
            room.mesh_use_embedded_materials = true;
            room.lifetime = rclcpp::Duration::from_seconds(0);
            room.type = visualization_msgs::msg::Marker::CUBE;
            roomArray.markers.push_back(room);

            // Room label (name)
            roomLabel.id = idx;
            roomLabel.color.a = 1;
            roomLabel.color.r = 0;
            roomLabel.color.g = 0;
            roomLabel.color.b = 0;
            roomLabel.scale.z = 0.2;
            roomLabel.text = roomName;
            roomLabel.ns = "roomLabel";
            roomLabel.action = roomLabel.ADD;
            roomLabel.header.stamp = msgTime;
            roomLabel.header.frame_id = frameSE;
            roomLabel.pose.position.x = centroid.x();
            roomLabel.pose.position.z = centroid.z();
            roomLabel.pose.position.y = centroid.y() + textOffset;
            roomLabel.lifetime = rclcpp::Duration::from_seconds(0);
            roomLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            roomArray.markers.push_back(roomLabel);

            // Room to Wall connection line
            roomWallLine.id = idx;
            roomWallLine.color.a = 0.9;
            roomWallLine.color.r = 0.0;
            roomWallLine.color.g = 0.0;
            roomWallLine.color.b = 0.0;
            roomWallLine.scale.x = 0.05;
            roomWallLine.scale.y = 0.05;
            roomWallLine.scale.z = 0.05;
            roomWallLine.ns = "roomWallLine";
            roomWallLine.header.stamp = msgTime;
            roomWallLine.action = roomWallLine.ADD;
            roomWallLine.header.frame_id = frameWorld;
            roomWallLine.lifetime = rclcpp::Duration::from_seconds(0);
            roomWallLine.type = visualization_msgs::msg::Marker::LINE_LIST;

            // Fill in the room center point
            roomPoint.header.stamp = msgTime;
            roomPoint.point.x = centroid.x();
            roomPoint.point.y = centroid.y();
            roomPoint.point.z = centroid.z();
            roomPoint.header.frame_id = frameSE;

            try
            {
                // Transform the room center point to the world frame
                auto tfStamped = tfBuffer_->lookupTransform(
                    frameWorld, frameSE, msgTime, rclcpp::Duration::from_seconds(0.1));
                tf2::doTransform(roomPoint, roomPointTr, tfStamped);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Room center transform failed: %s", ex.what());
                roomPointTr = roomPoint;
            }

            // Room to Wall connection line
            for (const auto wall : rooms[idx]->getWalls())
            {
                // Skip if the wall is bad
                if (wall->isBad())
                    continue;

                // Variables
                geometry_msgs::msg::Point pointRoom, pointWall;
                geometry_msgs::msg::PointStamped wallPoint, wallPointTr;

                pointRoom.x = roomPointTr.point.x;
                pointRoom.y = roomPointTr.point.y;
                pointRoom.z = roomPointTr.point.z;
                roomWallLine.points.push_back(pointRoom);

                wallPoint.header.stamp = msgTime;
                wallPoint.header.frame_id = frameBC;
                wallPoint.point.x = wall->getCentroid().x();
                wallPoint.point.y = wall->getCentroid().y();
                wallPoint.point.z = wall->getCentroid().z();

                try
                {
                    // Transform the room center point to the world frame
                    auto tfStamped = tfBuffer_->lookupTransform(
                        frameWorld, frameBC, msgTime, rclcpp::Duration::from_seconds(0.1));
                    tf2::doTransform(wallPoint, wallPointTr, tfStamped);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Wall centroid transform failed: %s", ex.what());
                    wallPointTr = wallPoint;
                }

                pointWall.x = wallPointTr.point.x;
                pointWall.y = wallPointTr.point.y;
                pointWall.z = wallPointTr.point.z;
                roomWallLine.points.push_back(pointWall);
            }

            // Add items to the roomArray
            roomArray.markers.push_back(roomWallLine);
        }

        pubStructuralElements->publish(roomArray);
    }

    // Publish floors, if any
    if (numFloors > 0)
    {
        // Variables
        std::vector<double> color = {0.3, 0.6, 0.7};

        // Loop through all the floors
        for (int floorId = 0; floorId < numFloors; floorId++)
        {
            // If the floor has no rooms, skip it
            if (floors[floorId]->getRooms().size() == 0)
                continue;

            // Variables
            std::string floorName = floors[floorId]->getName();
            geometry_msgs::msg::PointStamped floorPoint, floorPointTr;
            Eigen::Vector3d floorCentroid = floors[floorId]->getCentroid();
            visualization_msgs::msg::Marker floorMarker, floorLabel, floorRoomLine;

            // Floor marker (cube)
            floorMarker.id = floorId;
            floorMarker.ns = "floors";
            floorMarker.scale.x = 0.4;
            floorMarker.scale.y = 0.4;
            floorMarker.scale.z = 0.4;
            floorMarker.color.a = 1.0;
            floorMarker.color.r = color[0];
            floorMarker.color.g = color[1];
            floorMarker.color.b = color[2];
            floorMarker.header.stamp = msgTime;
            floorMarker.action = floorMarker.ADD;
            floorMarker.pose.orientation.x = 0.0;
            floorMarker.pose.orientation.y = 0.0;
            floorMarker.pose.orientation.z = 0.0;
            floorMarker.pose.orientation.w = 1.0;
            floorMarker.header.frame_id = frameSE;
            floorMarker.pose.position.x = floorCentroid.x();
            floorMarker.pose.position.y = floorCentroid.y();
            floorMarker.pose.position.z = floorCentroid.z();
            floorMarker.lifetime = rclcpp::Duration::from_seconds(0);
            floorMarker.type = visualization_msgs::msg::Marker::CUBE;

            // Floor label (name)
            floorLabel.color.a = 1;
            floorLabel.color.r = 0;
            floorLabel.color.g = 0;
            floorLabel.color.b = 0;
            floorLabel.scale.z = 0.2;
            floorLabel.id = floorId;
            floorLabel.text = floorName;
            floorLabel.ns = "floorLabels";
            floorLabel.header.stamp = msgTime;
            floorLabel.action = floorLabel.ADD;
            floorLabel.header.frame_id = frameSE;
            floorLabel.pose.position.x = floorCentroid.x();
            floorLabel.pose.position.y = floorCentroid.y();
            floorLabel.pose.position.z = floorCentroid.z();
            floorLabel.lifetime = rclcpp::Duration::from_seconds(0);
            floorLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            // Apply offset to the floor centroid for better visualization
            if (sensorType == ORB_SLAM3::System::IMU_RGBD)
            {
                floorMarker.pose.position.z += floorToRoomOffset;
                floorLabel.pose.position.z += (floorToRoomOffset + textOffset);
            }
            else
            {
                floorMarker.pose.position.y += floorToRoomOffset;
                floorLabel.pose.position.y += (floorToRoomOffset + textOffset);
            }

            // Floor to Room connection line
            floorRoomLine.id = floorId;
            floorRoomLine.color.a = 0.9;
            floorRoomLine.color.r = 0.0;
            floorRoomLine.color.g = 0.0;
            floorRoomLine.color.b = 0.0;
            floorRoomLine.scale.x = 0.05;
            floorRoomLine.scale.y = 0.05;
            floorRoomLine.scale.z = 0.05;
            floorRoomLine.ns = "floorRoomEdges";
            floorRoomLine.header.stamp = msgTime;
            floorRoomLine.action = floorRoomLine.ADD;
            floorRoomLine.header.frame_id = frameWorld;
            floorRoomLine.lifetime = rclcpp::Duration::from_seconds(0);
            floorRoomLine.type = visualization_msgs::msg::Marker::LINE_LIST;

            // Create point for the floor centroid in the world frame
            floorPoint.header.stamp = msgTime;
            floorPoint.header.frame_id = frameSE;
            floorPoint.point.x = floorMarker.pose.position.x;
            floorPoint.point.y = floorMarker.pose.position.y;
            floorPoint.point.z = floorMarker.pose.position.z;

            try
            {
                // Transform the room center point to the world frame
                auto tfStamped = tfBuffer_->lookupTransform(
                    frameWorld, frameSE, msgTime, rclcpp::Duration::from_seconds(0.1));
                tf2::doTransform(floorPoint, floorPointTr, tfStamped);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Floor center transform failed: %s", ex.what());
                floorPointTr = floorPoint;
            }

            // Connect the floor to its rooms
            for (const auto room : floors[floorId]->getRooms())
            {
                geometry_msgs::msg::Point pFloor, pRoom;
                geometry_msgs::msg::PointStamped roomPoint, roomPointTr;

                pFloor.x = floorPointTr.point.x;
                pFloor.y = floorPointTr.point.y;
                pFloor.z = floorPointTr.point.z;
                floorRoomLine.points.push_back(pFloor);

                roomPoint.header.stamp = msgTime;
                roomPoint.header.frame_id = frameSE;
                roomPoint.point.x = room->getCentroid().x();
                roomPoint.point.y = room->getCentroid().y();
                roomPoint.point.z = room->getCentroid().z();

                try
                {
                    // Transform the room center point to the world frame
                    auto tfStamped = tfBuffer_->lookupTransform(
                        frameWorld, frameSE, msgTime, rclcpp::Duration::from_seconds(0.1));
                    tf2::doTransform(roomPoint, roomPointTr, tfStamped);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Room centroid transform failed: %s", ex.what());
                    roomPointTr = roomPoint;
                }

                pRoom.x = roomPointTr.point.x;
                pRoom.y = roomPointTr.point.y;
                pRoom.z = roomPointTr.point.z;

                floorRoomLine.points.push_back(pRoom);
            }

            // Add the floor marker
            floorArray.markers.push_back(floorLabel);
            floorArray.markers.push_back(floorMarker);
            floorArray.markers.push_back(floorRoomLine);
        }

        pubStructuralElements->publish(floorArray);
    }
}

sensor_msgs::msg::PointCloud2 mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints, rclcpp::Time msgTime)
{
    const int numChannels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    std::string channelId[] = {"x", "y", "z"};

    // Set the attributes of the point cloud
    cloud.header.stamp = msgTime;
    cloud.header.frame_id = frameWorld;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    cloud.width = mapPoints.size();
    cloud.point_step = numChannels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(numChannels);

    // Set the fields of the point cloud
    for (int idx = 0; idx < numChannels; idx++)
    {
        cloud.fields[idx].count = 1;
        cloud.fields[idx].name = channelId[idx];
        cloud.fields[idx].offset = idx * sizeof(float);
        cloud.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    // Set the data of the point cloud
    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloudDataPtr = &(cloud.data[0]);

    // Populate the point cloud with the map points
    for (unsigned int idx = 0; idx < cloud.width; idx++)
    {
        if (mapPoints[idx] && !mapPoints[idx]->isBad())
        {
            Eigen::Vector3d P3Dw = mapPoints[idx]->GetWorldPos().cast<double>();
            tf2::Vector3 pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());
            float dataArray[numChannels] = {
                static_cast<float>(pointTranslation.x()),
                static_cast<float>(pointTranslation.y()),
                static_cast<float>(pointTranslation.z())};
            memcpy(cloudDataPtr + (idx * cloud.point_step), dataArray, numChannels * sizeof(float));
        }
    }

    return cloud;
}

cv::Mat SE3fToCvMat(Sophus::SE3f data)
{
    cv::Mat cvMat;

    // Convert the Eigen matrix to OpenCV matrix
    Eigen::Matrix4f T_Eig3f = data.matrix();
    cv::eigen2cv(T_Eig3f, cvMat);

    return cvMat;
}

// tf::Transform SE3fToTFTransform(Sophus::SE3f data)
tf2::Transform SE3fToTFTransform(Sophus::SE3f data)
{
    Eigen::Matrix3f rotMatrix = data.rotationMatrix();
    Eigen::Vector3f transVector = data.translation();

    tf2::Matrix3x3 rotationTF(
        rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2),
        rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2),
        rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2));

    tf2::Vector3 translationTF(
        transVector(0),
        transVector(1),
        transVector(2));

    return tf2::Transform(rotationTF, translationTF);
}

// void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray)
// {
//     // The list of markers observed in the current frame
//     std::vector<ORB_SLAM3::Marker *> currentMarkers;

//     // Process the received marker array
//     for (const auto &marker : markerArray.markers)
//     {
//         // Access information of each passed ArUco marker
//         int markerId = marker.id;
//         double visitTime = marker.header.stamprclcpp::Time::seconds();
//         geometry_msgs::Pose markerPose = marker.pose.pose;
//         geometry_msgs::msg::Point markerPosition = markerPose.position;            // (x,y,z)
//         geometry_msgs::Quaternion markerOrientation = markerPose.orientation; // (x,y,z,w)

//         Eigen::Vector3f markerTranslation(markerPosition.x, markerPosition.y, markerPosition.z);
//         Eigen::Quaterniond markerQuaternion(markerOrientation.w, markerOrientation.x,
//                                             markerOrientation.y, markerOrientation.z);
//         Sophus::SE3f normalizedPose(markerQuaternion, markerTranslation);

//         // Create a marker object of the currently visited marker
//         ORB_SLAM3::Marker *currentMarker = new ORB_SLAM3::Marker();
//         currentMarker->setOpId(-1);
//         currentMarker->setId(markerId);
//         currentMarker->setTime(visitTime);
//         currentMarker->setMarkerInGMap(false);
//         currentMarker->setLocalPose(normalizedPose);
//         currentMarker->setMarkerType(ORB_SLAM3::Marker::markerVariant::UNKNOWN);

//         // Add it to the list of observed markers
//         currentMarkers.push_back(currentMarker);
//     }

//     // Add the new markers to the list of markers in buffer
//     if (currentMarkers.size() > 0)
//         markersBuffer.push_back(currentMarkers);
// }

std::pair<double, std::vector<ORB_SLAM3::Marker *>> findNearestMarker(double frameTimestamp)
{
    double minTimeDifference = 100;
    std::vector<ORB_SLAM3::Marker *> matchedMarkers;

    // Loop through the markersBuffer
    for (const auto &markers : markersBuffer)
    {
        double timeDifference = markers[0]->getTime() - frameTimestamp;
        if (timeDifference < minTimeDifference)
        {
            matchedMarkers = markers;
            minTimeDifference = timeDifference;
        }
    }

    return std::make_pair(minTimeDifference, matchedMarkers);
}

void setVoxbloxSkeletonCluster(const visualization_msgs::msg::MarkerArray &skeletonArray)
{
    // Reset the buffer
    skeletonClusterPoints.clear();

    for (const auto &skeleton : skeletonArray.markers)
    {
        // Take the points of the current cluster
        std::vector<Eigen::Vector3d> clusterPoints;

        // Pick only the messages starting with name "connected_vertices_[x]"
        if (skeleton.ns.compare(0, strlen("connected_vertices"), "connected_vertices") == 0)
        {
            // Skip small clusters
            if (skeleton.points.size() > ORB_SLAM3::SystemParams::GetParams()->room_seg.min_cluster_vertices)
                // Add the points of the cluster to the buffer
                for (const auto &point : skeleton.points)
                {
                    // transform from map frame to world frame
                    geometry_msgs::msg::PointStamped pointIn, pointOut;
                    pointIn.header.frame_id = frameMap;
                    pointIn.header.stamp = rclcpp::Time(0);
                    pointIn.point = point;
                    try
                    {
                        auto tf_stamped = tfBuffer_->lookupTransform(
                            frameWorld, pointIn.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
                        tf2::doTransform(pointIn, pointOut, tf_stamped);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Could not transform skeleton cluster point: %s", ex.what());
                        pointOut = pointIn; // Fallback: use original point
                    }
                    // Add the point to the cluster
                    Eigen::Vector3d newPoint(pointOut.point.x, pointOut.point.y, pointOut.point.z);
                    clusterPoints.push_back(newPoint);
                }

            // Add the current cluster to the skeleton cluster points buffer
            if (clusterPoints.size() > 0)
                skeletonClusterPoints.push_back(clusterPoints);
        }
    }

    // Set the cluster points to the active map
    pSLAM->setSkeletonCluster(skeletonClusterPoints);
}

void setGNNBasedRoomCandidates(const situational_graphs_msgs::msg::RoomsData &msgGNNRooms)
{
    // Reset the buffer
    gnnRoomCandidates.clear();

    // Loop through the received GNN rooms
    // for (const auto &room : msgGNNRooms.rooms)
    // {
    //     // Create a new room object
    //     ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

    //     // Set the room properties
    //     newRoom->setId(room.id);
    //     newRoom->setHasKnownLabel(false);

    //     // [TODO] use room.wallIds to fill newRoom->setWalls
    //     // [TODO] use room.centroid to fill newRoom->setRoomCenter

    //     // Add the room to the GNN candidates buffer
    //     gnnRoomCandidates.push_back(newRoom);

    //     // [TODO] Add, update, and remove the room in the GNN-based room candidates
    // }

    // [TODO] Define a 'setGNNRoomCandidates' in System.h
    pSLAM->setGNNRoomCandidates(gnnRoomCandidates);
}

void setGNNBasedRoomCandidates(const vs_graphs::msg::VSGraphsAllDetectdetRooms &msgGNNRooms)
{
    // Reset the buffer
    gnnRoomCandidates.clear();

    // Loop through the received GNN rooms
    for (const auto &room : msgGNNRooms.rooms)
    {
        // Create a new room object
        ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

        // Set the room properties
        newRoom->setId(room.id);
        newRoom->setHasKnownLabel(false);

        // [TODO] use room.wallIds to fill newRoom->setWalls
        // [TODO] use room.centroid to fill newRoom->setRoomCenter

        // Add the room to the GNN candidates buffer
        gnnRoomCandidates.push_back(newRoom);

        // [TODO] Add, update, and remove the room in the GNN-based room candidates
    }

    // [TODO] Define a 'setGNNRoomCandidates' in System.h
    pSLAM->setGNNRoomCandidates(gnnRoomCandidates);
}