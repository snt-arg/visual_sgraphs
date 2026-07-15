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
#include <cmath>
#include <set>
#include <unordered_map>
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

double lastPlanePublishTime = -std::numeric_limits<double>::infinity();
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubKeyFrameList;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubDoor;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAllMappoints;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubCameraPose;
rclcpp::Publisher<segmenter_ros::msg::VSGraphDataMsg>::SharedPtr pubKFImage;
rclcpp::Publisher<keyframe_depth_estimator::msg::KeyFrameCreated>::SharedPtr pubKeyFrameCreated;
rclcpp::Publisher<keyframe_depth_validator::msg::StaticMapPointCorrespondences>::SharedPtr pubKeyFrameStaticMapPoints;
rclcpp::Publisher<keyframe_depth_validator::msg::StaticMapPoints>::SharedPtr pubStaticMapPoints;
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
rclcpp::Publisher<vs_graphs::msg::MapResetEvent>::SharedPtr pubMapReset;
rclcpp::Publisher<vs_graphs::msg::MapReadyEvent>::SharedPtr pubMapReady;
rclcpp::Publisher<vs_graphs::msg::MapRescaleEvent>::SharedPtr pubMapRescale;

// Marker ids published in the previous cycle, keyed by RViz marker namespace. An entity stops
// appearing in the SLAM system's GetAllX() lists either because it was individually pruned, or
// because its owning map is no longer the active one (e.g. after a tracking-loss reset) -- in
// both cases the corresponding marker(s) must be explicitly DELETEd, since marker.lifetime is 0
// (forever) and RViz otherwise keeps drawing them indefinitely.
std::unordered_map<std::string, std::set<int>> lastPublishedMarkerIds;

// Appends DELETE markers for any id previously published under `ns` that is absent from
// `currentIds` this cycle, then records `currentIds` as the new baseline for `ns`.
void appendDeletedMarkers(
    visualization_msgs::msg::MarkerArray &markerArray,
    const std::string &ns,
    const std::set<int> &currentIds,
    const std::string &frameId,
    const rclcpp::Time &stamp)
{
    std::set<int> &previousIds = lastPublishedMarkerIds[ns];
    for (int id : previousIds)
    {
        if (currentIds.count(id))
            continue;

        visualization_msgs::msg::Marker deleteMarker;
        deleteMarker.ns = ns;
        deleteMarker.id = id;
        deleteMarker.header.frame_id = frameId;
        deleteMarker.header.stamp = stamp;
        deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
        markerArray.markers.push_back(deleteMarker);
    }
    previousIds = currentIds;
}

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

void publishKeyFrameCreatedEvent(const ORB_SLAM3::KeyFrame *keyframe)
{
    if (!pubKeyFrameCreated || keyframe == nullptr || keyframe->mImage.empty())
        return;

    try
    {
        cv::Mat keyframeImageBgr;
        if (keyframe->mImage.channels() == 1)
            cv::cvtColor(keyframe->mImage, keyframeImageBgr, cv::COLOR_GRAY2BGR);
        else if (keyframe->mImage.channels() == 4)
            cv::cvtColor(keyframe->mImage, keyframeImageBgr, cv::COLOR_BGRA2BGR);
        else if (keyframe->mImage.channels() == 3)
            keyframeImageBgr = keyframe->mImage;
        else
            return;

        std_msgs::msg::Header header;
        header.stamp = rclcpp::Time(static_cast<int64_t>(keyframe->mTimeStamp * 1e9));
        header.frame_id = frameCamera;

        keyframe_depth_estimator::msg::KeyFrameCreated event;
        event.header = header;
        event.keyframe_id = keyframe->mnId;
        auto image_msg = cv_bridge::CvImage(
            header, sensor_msgs::image_encodings::BGR8, keyframeImageBgr).toImageMsg();
        event.image = *image_msg;
        pubKeyFrameCreated->publish(event);

        if (pubKeyFrameStaticMapPoints)
        {
            keyframe_depth_validator::msg::StaticMapPointCorrespondences points_msg;
            points_msg.header = header;
            points_msg.keyframe_id = keyframe->mnId;

            ORB_SLAM3::KeyFrame *mutable_keyframe = const_cast<ORB_SLAM3::KeyFrame *>(keyframe);
            const std::vector<ORB_SLAM3::MapPoint *> map_points =
                mutable_keyframe->GetMapPointMatches();
            const Sophus::SE3f Tcw = mutable_keyframe->GetPose();
            points_msg.u.reserve(map_points.size());
            points_msg.v.reserve(map_points.size());
            points_msg.z_orb.reserve(map_points.size());
            points_msg.map_point_ids.reserve(map_points.size());

            for (std::size_t i = 0; i < map_points.size() && i < keyframe->mvKeysUn.size(); ++i)
            {
                ORB_SLAM3::MapPoint *map_point = map_points[i];
                if (map_point == nullptr || map_point->isBad())
                    continue;

                const cv::Point2f pixel = keyframe->mvKeysUn[i].pt;
                if (pixel.x < 0.0F || pixel.y < 0.0F ||
                    pixel.x >= static_cast<float>(keyframeImageBgr.cols) ||
                    pixel.y >= static_cast<float>(keyframeImageBgr.rows))
                    continue;

                const Eigen::Vector3f Xc = Tcw * map_point->GetWorldPos();
                if (!std::isfinite(Xc.z()) || Xc.z() <= 0.0F)
                    continue;

                points_msg.u.push_back(pixel.x);
                points_msg.v.push_back(pixel.y);
                points_msg.z_orb.push_back(Xc.z());
                points_msg.map_point_ids.push_back(static_cast<uint64_t>(map_point->mnId));
            }

            pubKeyFrameStaticMapPoints->publish(points_msg);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Failed to publish KeyFrameCreated event for keyframe %lu: %s",
                     keyframe->mnId, e.what());
    }
}

void setupPublishers(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<image_transport::ImageTransport> image_transport, const std::string &node_name)
{
    // Basic
    pubKeyFrameList = node->create_publisher<nav_msgs::msg::Path>(node_name + "/keyframe_list", 2);
    pubAllMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", 1);
    pubCameraPose = node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", 1);
    pubKFImage = node->create_publisher<segmenter_ros::msg::VSGraphDataMsg>(node_name + "/keyframe_image", 50);
    pubKeyFrameCreated =
        node->create_publisher<keyframe_depth_estimator::msg::KeyFrameCreated>(
            "/orbslam3/keyframe_created", 10);
    pubKeyFrameStaticMapPoints =
        node->create_publisher<keyframe_depth_validator::msg::StaticMapPointCorrespondences>(
            "/orbslam3/keyframe_static_map_points", 10);
    pubStaticMapPoints =
        node->create_publisher<keyframe_depth_validator::msg::StaticMapPoints>(
            "/orbslam3/static_map_points", 10);
    pubTrackedMappoints = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", 1);
    pubWorldFramePointCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/points_map", 1);
    pubKeyFrameMarker = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/kf_markers", 1);
    pubFreespaceCluster = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/freespace_clusters", 1);
    pubCameraPoseVis = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/camera_pose_vis", 1);
    pubTrackingImage = std::make_shared<image_transport::Publisher>(image_transport->advertise(node_name + "/tracking_image", 1));

    // Entities
    pubDoor = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/doors", 1);
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

    // Map reset notifications, for downstream nodes that keep their own per-map state
    pubMapReset = node->create_publisher<vs_graphs::msg::MapResetEvent>(node_name + "/map_reset", 1);

    // Map readiness (scale/IMU initialized), so downstream nodes know when it's safe to start
    // creating entities/tracks. Unlike the other publishers here, this uses transient_local
    // ("latched") durability: readiness is a level, not a pure edge, so a node that (re)connects
    // later must learn the current state immediately instead of waiting for the next transition.
    pubMapReady = node->create_publisher<vs_graphs::msg::MapReadyEvent>(
        node_name + "/map_ready", rclcpp::QoS(1).transient_local());

    // Map rescale notifications (scale/gravity refinement, or a map merge), so downstream nodes
    // that buffer their own world-frame positions/poses can apply the same correction instead of
    // going stale relative to the (now rescaled) map.
    pubMapRescale = node->create_publisher<vs_graphs::msg::MapRescaleEvent>(node_name + "/map_rescale", 1);

    // Get body odometry if IMU data is also available
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR || sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
        pubOdometry = node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", 1);

    if (pSLAM)
        pSLAM->SetKeyFrameCreatedCallback(publishKeyFrameCreatedEvent);

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

// Announces when the current active map changes (tracking-loss reset, new map created), so that
// downstream nodes keeping their own per-map/per-track state (e.g. object_motion_estimator) can
// clear it instead of silently mixing data across two unrelated coordinate frames. Detecting this
// needs no core changes: Map::GetId() is already a stable, globally unique, monotonically
// increasing id assigned once per Map object.
void publishMapResetIfChanged(rclcpp::Time msgTime)
{
    static uint64_t lastMapId = std::numeric_limits<uint64_t>::max(); // sentinel: nothing seen yet
    static uint64_t resetCount = 0;

    uint64_t currentMapId = pSLAM->GetCurrentMap()->GetId();
    if (currentMapId == lastMapId)
        return;

    // Only the transition to a *different already-seen-once* map is a reset; the very first map
    // observed at startup is not.
    if (lastMapId != std::numeric_limits<uint64_t>::max())
    {
        vs_graphs::msg::MapResetEvent event;
        event.header.stamp = msgTime;
        event.header.frame_id = frameWorld;
        event.previous_map_id = lastMapId;
        event.current_map_id = currentMapId;
        event.reset_count = ++resetCount;
        pubMapReset->publish(event);
    }
    lastMapId = currentMapId;
}

// Announces whether the current map is safe to create entities/tracks against -- non-inertial
// sensors are always ready once a map exists (no separate scale phase); inertial sensors aren't
// ready until GetIniertialBA2() (scale/gravity actually converged, not just the earliest
// bias-only isImuInitialized() stage -- mirrors the core-side guard in Atlas::AddMapDoor etc. and
// the existing precedent for this exact pairing in Tracking.cc). Publishes once per map (whatever
// the state is at first sight) and again only on each actual true/false flip thereafter.
void publishMapReadyIfChanged(rclcpp::Time msgTime)
{
    static uint64_t lastMapId = std::numeric_limits<uint64_t>::max();
    static bool lastPublishedReady = false;
    static bool havePublishedForThisMap = false;

    ORB_SLAM3::Map *pMap = pSLAM->GetCurrentMap();
    uint64_t currentMapId = pMap->GetId();
    bool currentlyReady = !pMap->IsInertial() || (pMap->isImuInitialized() && pMap->GetIniertialBA2());

    if (currentMapId != lastMapId)
    {
        lastMapId = currentMapId;
        havePublishedForThisMap = false;
    }
    if (havePublishedForThisMap && currentlyReady == lastPublishedReady)
        return;

    vs_graphs::msg::MapReadyEvent event;
    event.header.stamp = msgTime;
    event.header.frame_id = frameWorld;
    event.map_id = currentMapId;
    event.is_ready = currentlyReady;
    pubMapReady->publish(event);
    lastPublishedReady = currentlyReady;
    havePublishedForThisMap = true;
}

// Announces the similarity correction (scale, rotation, translation) whenever
// Map::ApplyScaledRotation runs (IMU scale/gravity refinement, or a map merge) -- downstream nodes
// that buffer their own world-frame positions/poses (e.g. object_motion_estimator) apply the same
// correction to stay consistent with the now-rescaled map. Uses Map::GetScaleCorrectionVersion(),
// a dedicated counter -- unlike GetMapChangeIndex()/mnMapChange, which is bumped by many unrelated
// events (BA, loop closure, merge welding) and would give false positives here.
void publishMapRescaleIfChanged(rclcpp::Time msgTime)
{
    static uint64_t lastMapId = std::numeric_limits<uint64_t>::max();
    static uint64_t lastVersion = 0;

    ORB_SLAM3::Map *pMap = pSLAM->GetCurrentMap();
    uint64_t currentMapId = pMap->GetId();
    if (currentMapId != lastMapId)
    {
        // A new map's version counter starts back at 0 -- reset our bookkeeping too, so a new
        // map's first rescale isn't skipped just because its version happens to match whatever
        // the previous map's counter last was.
        lastMapId = currentMapId;
        lastVersion = 0;
    }

    uint64_t currentVersion = pMap->GetScaleCorrectionVersion();
    if (currentVersion == lastVersion)
        return;
    lastVersion = currentVersion;
    if (currentVersion == 0)
        return; // no rescale has happened yet for this map

    Sophus::SE3f T;
    float s;
    pMap->GetLastScaleCorrection(T, s);

    vs_graphs::msg::MapRescaleEvent event;
    event.header.stamp = msgTime;
    event.header.frame_id = frameWorld;
    event.map_id = pMap->GetId();
    event.scale = static_cast<double>(s);
    event.rotation.x = T.unit_quaternion().x();
    event.rotation.y = T.unit_quaternion().y();
    event.rotation.z = T.unit_quaternion().z();
    event.rotation.w = T.unit_quaternion().w();
    event.translation.x = T.translation().x();
    event.translation.y = T.translation().y();
    event.translation.z = T.translation().z();
    event.rescale_count = currentVersion;
    pubMapRescale->publish(event);
}

void publishTopics(rclcpp::Time msgTime, Eigen::Vector3f Wbb, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msgPCL)
{
    // Detect and announce a map reset/readiness/rescale change as early as possible in the cycle,
    // even if the current pose is temporarily invalid (NaN, handled below) -- downstream consumers
    // need to know their world-frame assumptions broke (or that it's now safe to resume, or that a
    // correction needs applying) right away, not only once tracking recovers.
    publishMapResetIfChanged(msgTime);
    publishMapReadyIfChanged(msgTime);
    publishMapRescaleIfChanged(msgTime);

    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    // Avoid publishing NaN
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0))
        return;

    // Common topics
    publishCameraPose(Twc, msgTime);
    publishStaticMapPoints(pSLAM->GetTrackedMapPoints(), msgTime);
    publishTFTransform(Twc, frameWorld, frameCamera, msgTime);
    publishFramePointCloud(Twc, msgPCL, msgTime);

    // Set a static transform between the world and map frame
    if (pubStaticTransform)
        publishStaticTFTransform(frameWorld, frameMap, msgTime);

    // Get KeyFrames
    std::vector<ORB_SLAM3::KeyFrame *> keyframes = pSLAM->GetAllKeyFrames();

    // Setup publishers
    publishDoors(pSLAM->GetAllDoors());
    publishKeyFrameImages(keyframes, msgTime);
    publishKeyFrameMarkers(keyframes, msgTime);
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

        if (keyframe->mImage.empty())
            continue;

        // Create an object of VSGraphDataMsg
        segmenter_ros::msg::VSGraphDataMsg vsGraphPublisher = segmenter_ros::msg::VSGraphDataMsg();
        std_msgs::msg::Header header;
        header.stamp = msgTime;
        header.frame_id = frameWorld;
        std_msgs::msg::UInt64 kfId;
        kfId.data = keyframe->mnId;
        cv::Mat keyframeImageBgr;
        if (keyframe->mImage.channels() == 1)
            cv::cvtColor(keyframe->mImage, keyframeImageBgr, cv::COLOR_GRAY2BGR);
        else if (keyframe->mImage.channels() == 4)
            cv::cvtColor(keyframe->mImage, keyframeImageBgr, cv::COLOR_BGRA2BGR);
        else if (keyframe->mImage.channels() == 3)
            keyframeImageBgr = keyframe->mImage;
        else
            continue;

        const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, keyframeImageBgr).toImageMsg();

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

void publishStaticMapPoints(std::vector<ORB_SLAM3::MapPoint *> trackedMapPoints, rclcpp::Time msgTime)
{
    if (!pubStaticMapPoints)
        return;

    keyframe_depth_validator::msg::StaticMapPoints points_msg;
    points_msg.header.stamp = msgTime;
    points_msg.header.frame_id = frameWorld;

    const std::vector<cv::KeyPoint> &trackedKeyPointsUn = pSLAM->GetTrackedKeyPointsUn();
    const Sophus::SE3f Tcw = pSLAM->GetCamTwc().inverse();

    const std::size_t count = std::min(trackedMapPoints.size(), trackedKeyPointsUn.size());
    points_msg.point_ids.reserve(count);
    points_msg.positions_world.reserve(count);
    points_msg.pixels.reserve(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        ORB_SLAM3::MapPoint *map_point = trackedMapPoints[i];
        if (map_point == nullptr || map_point->isBad())
            continue;

        const Eigen::Vector3f pos_world = map_point->GetWorldPos();
        const Eigen::Vector3f p_cam = Tcw * pos_world;
        if (!std::isfinite(p_cam.z()) || p_cam.z() <= 0.0F)
            continue;

        geometry_msgs::msg::Point p_world_msg;
        p_world_msg.x = static_cast<double>(pos_world.x());
        p_world_msg.y = static_cast<double>(pos_world.y());
        p_world_msg.z = static_cast<double>(pos_world.z());

        geometry_msgs::msg::Point p_pixel_msg;
        p_pixel_msg.x = static_cast<double>(trackedKeyPointsUn[i].pt.x);
        p_pixel_msg.y = static_cast<double>(trackedKeyPointsUn[i].pt.y);
        p_pixel_msg.z = 0.0;

        points_msg.point_ids.push_back(static_cast<uint32_t>(map_point->mnId));
        points_msg.positions_world.push_back(p_world_msg);
        points_msg.pixels.push_back(p_pixel_msg);
    }

    pubStaticMapPoints->publish(points_msg);
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

    // kf_markers is a single SPHERE_LIST marker (fixed id) whose points are fully replaced
    // below, so an empty keyframe_vec (e.g. right after a map reset) must still be published
    // -- with an empty points list -- rather than skipped, otherwise the previous map's whole
    // trajectory marker is left stuck on screen forever (marker lifetime is 0).
    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = frameWorld;
    kf_markers.header.stamp = msgTime;
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
    visualization_msgs::msg::MarkerArray markerArray;
    std::set<int> currentIds;
    rclcpp::Time now = rclcpp::Clock().now(); // rclcpp::Time().now();

    for (auto *marker : markers)
    {
        int id = marker->getId();
        currentIds.insert(id);

        visualization_msgs::msg::Marker fiducial_marker;
        Sophus::SE3f markerPose = marker->getGlobalPose();

        fiducial_marker.color.a = 0;
        fiducial_marker.scale.x = 0.2;
        fiducial_marker.scale.y = 0.2;
        fiducial_marker.scale.z = 0.2;
        fiducial_marker.ns = "fiducial_markers";
        fiducial_marker.lifetime = rclcpp::Duration::from_seconds(0);
        fiducial_marker.action = fiducial_marker.ADD;
        fiducial_marker.id = id;
        fiducial_marker.header.stamp = now;
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

    appendDeletedMarkers(markerArray, "fiducial_markers", currentIds, frameBC, now);
    pubFiducialMarker->publish(markerArray);
}

void publishDoors(std::vector<ORB_SLAM3::Door *> doors)
{
    // Variables
    visualization_msgs::msg::MarkerArray doorArray;
    std::set<int> currentIds;
    rclcpp::Time now = rclcpp::Clock().now();

    for (auto *doorPtr : doors)
    {
        int id = doorPtr->getId();
        currentIds.insert(id);

        Sophus::SE3f doorPose = doorPtr->getGlobalPose();
        visualization_msgs::msg::Marker door, doorLines, doorLabel;

        // Door values
        door.color.a = 0;
        door.ns = "doors";
        door.scale.x = 0.5;
        door.scale.y = 0.5;
        door.scale.z = 0.5;
        door.action = door.ADD;
        door.lifetime = rclcpp::Duration::from_seconds(0);
        door.id = id;
        door.header.stamp = now; // rclcpp::Time().now();
        door.mesh_use_embedded_materials = true;
        door.header.frame_id = frameBC;
        door.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        door.mesh_resource =
            "package://vs_graphs/config/Assets/door.dae";

        // Rotation and displacement for better visualization
        Sophus::SE3f rotatedDoorPose = doorPose * Sophus::SE3f::rotX(-M_PI_2);
        rotatedDoorPose.translation().y() -= 1.0;
        door.pose.position.x = rotatedDoorPose.translation().x();
        door.pose.position.y = rotatedDoorPose.translation().y();
        door.pose.position.z = rotatedDoorPose.translation().z();
        door.pose.orientation.x = rotatedDoorPose.unit_quaternion().x();
        door.pose.orientation.y = rotatedDoorPose.unit_quaternion().y();
        door.pose.orientation.z = rotatedDoorPose.unit_quaternion().z();
        door.pose.orientation.w = rotatedDoorPose.unit_quaternion().w();
        doorArray.markers.push_back(door);

        // Door label (name)
        doorLabel.color.a = 1;
        doorLabel.color.r = 0;
        doorLabel.color.g = 0;
        doorLabel.color.b = 0;
        doorLabel.scale.z = 0.2;
        doorLabel.ns = "doorLabel";
        doorLabel.action = doorLabel.ADD;
        doorLabel.lifetime = rclcpp::Duration::from_seconds(0);
        doorLabel.text = doorPtr->getName();
        doorLabel.id = id;
        doorLabel.header.stamp = now; // rclcpp::Time().now();
        doorLabel.header.frame_id = frameBC;
        doorLabel.pose.position.x = door.pose.position.x;
        doorLabel.pose.position.z = door.pose.position.z;
        doorLabel.pose.position.y = door.pose.position.y - 1.2;
        doorLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        doorArray.markers.push_back(doorLabel);

        // Door to points connection line
        doorLines.color.a = 0.5;
        doorLines.color.r = 0.0;
        doorLines.color.g = 0.0;
        doorLines.color.b = 0.0;
        doorLines.scale.x = 0.005;
        doorLines.scale.y = 0.005;
        doorLines.scale.z = 0.005;
        doorLines.ns = "doorLines";
        doorLines.action = doorLines.ADD;
        doorLines.lifetime = rclcpp::Duration::from_seconds(0);
        doorLines.id = id;
        doorLines.header.stamp = now; // rclcpp::Time().now();
        doorLines.header.frame_id = frameBC;
        doorLines.type = visualization_msgs::msg::Marker::LINE_LIST;

        geometry_msgs::msg::Point point1;
        point1.x = doorPtr->getMarker()->getGlobalPose().translation().x();
        point1.y = doorPtr->getMarker()->getGlobalPose().translation().y();
        point1.z = doorPtr->getMarker()->getGlobalPose().translation().z();
        doorLines.points.push_back(point1);

        geometry_msgs::msg::Point point2;
        point2.x = rotatedDoorPose.translation().x();
        point2.y = rotatedDoorPose.translation().y();
        point2.z = rotatedDoorPose.translation().z();
        doorLines.points.push_back(point2);

        doorArray.markers.push_back(doorLines);
    }

    appendDeletedMarkers(doorArray, "doors", currentIds, frameBC, now);
    appendDeletedMarkers(doorArray, "doorLabel", currentIds, frameBC, now);
    appendDeletedMarkers(doorArray, "doorLines", currentIds, frameBC, now);
    pubDoor->publish(doorArray);
}

void publishPlanes(std::vector<ORB_SLAM3::Plane *> planes, rclcpp::Time msgTime)
{
    // Check if sufficient time has passed since the last plane publication
    if (msgTime.seconds() - lastPlanePublishTime < 3.0)
        return;

    lastPlanePublishTime = msgTime.seconds();

    // Variables
    visualization_msgs::msg::MarkerArray planeLabelArray;
    std::set<int> currentPlaneIds;
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

        // If the plane is undefined, skip it (it is left out of currentPlaneIds, so any
        // previously-published label/normal for it is auto-deleted by appendDeletedMarkers)
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
            continue;

        // Get the point clouds for the plane
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeClouds = plane->getMapClouds();
        if (planeClouds == nullptr || planeClouds->empty())
            continue;

        currentPlaneIds.insert(plane->getId());

        for (const auto &point : planeClouds->points)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x;
            newPoint.y = point.y;
            newPoint.z = point.z;
            newPoint.r = point.r;
            newPoint.g = point.g;
            newPoint.b = point.b;

            // Override color according to type of plane
            if (colorPointcloud)
            {
                newPoint.r = color[0];
                newPoint.g = color[1];
                newPoint.b = color[2];
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
        planeNormal.id = plane->getId();
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

    appendDeletedMarkers(planeLabelArray, "planeLabel", currentPlaneIds, frameBC, msgTime);
    appendDeletedMarkers(planeLabelArray, "planeNormal", currentPlaneIds, frameBC, msgTime);
    pubPlaneLabel->publish(planeLabelArray);

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
}

void publishStructuralElements(std::vector<ORB_SLAM3::Room *> rooms,
                               std::vector<ORB_SLAM3::Floor *> floors, rclcpp::Time msgTime)
{
    int numRooms = rooms.size();
    int numFloors = floors.size();

    // Variables
    double textOffset = -0.5;
    double floorToRoomOffset = -2.0;

    // Visualization markers
    visualization_msgs::msg::MarkerArray roomArray, floorArray;
    std::set<int> currentRoomIds, currentFloorIds;

    // Publish rooms, if any
    for (int idx = 0; idx < numRooms; idx++)
    {
            // A bad room is simply left out of currentRoomIds below, so it is
            // auto-deleted by appendDeletedMarkers once it drops out of the room list
            if (rooms[idx]->isBad())
                continue;

            int id = rooms[idx]->getId();
            currentRoomIds.insert(id);

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
            visualization_msgs::msg::Marker room, roomWallLine, roomDoorLine, roomMarkerLine, roomLabel;

            // Room values
            room.id = id;
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
            roomLabel.id = id;
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
            roomWallLine.id = id;
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

    appendDeletedMarkers(roomArray, "room", currentRoomIds, frameSE, msgTime);
    appendDeletedMarkers(roomArray, "roomLabel", currentRoomIds, frameSE, msgTime);
    appendDeletedMarkers(roomArray, "roomWallLine", currentRoomIds, frameWorld, msgTime);
    pubStructuralElements->publish(roomArray);

    // Publish floors, if any
    {
        // Variables
        std::vector<double> color = {0.3, 0.6, 0.7};

        // Loop through all the floors
        for (int floorId = 0; floorId < numFloors; floorId++)
        {
            // If the floor has no rooms, skip it, so it is auto-deleted by appendDeletedMarkers
            // below once it drops out of currentFloorIds
            if (floors[floorId]->getRooms().size() == 0)
                continue;

            int id = floors[floorId]->getId();
            currentFloorIds.insert(id);

            // Variables
            std::string floorName = floors[floorId]->getName();
            geometry_msgs::msg::PointStamped floorPoint, floorPointTr;
            Eigen::Vector3d floorCentroid = floors[floorId]->getCentroid();
            visualization_msgs::msg::Marker floorMarker, floorLabel, floorRoomLine;

            // Floor marker (cube)
            floorMarker.id = id;
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
            floorLabel.id = id;
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
            floorRoomLine.id = id;
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
    }

    appendDeletedMarkers(floorArray, "floors", currentFloorIds, frameSE, msgTime);
    appendDeletedMarkers(floorArray, "floorLabels", currentFloorIds, frameSE, msgTime);
    appendDeletedMarkers(floorArray, "floorRoomEdges", currentFloorIds, frameWorld, msgTime);
    pubStructuralElements->publish(floorArray);
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
//         Eigen::Quaternionf markerQuaternion(markerOrientation.w, markerOrientation.x,
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
