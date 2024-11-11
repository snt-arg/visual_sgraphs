/**
 * 🚀 [vS-Graphs] Common Functions and Variables Across All Modes (mono/stereo, with or w/o imu)*
 */

#include "common.h"

// Variables for ORB-SLAM3
ORB_SLAM3::System *pSLAM;
ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
bool colorPointcloud = true;
double roll = 0, pitch = 0, yaw = 0;
bool pubStaticTransform, pubPointClouds;
image_transport::Publisher pubTrackingImage;
rviz_visual_tools::RvizVisualToolsPtr visualTools;
std::shared_ptr<tf::TransformListener> transformListener;
std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;
std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;
ros::Publisher pubCameraPose, pubCameraPoseVis, pubOdometry, pubKeyFrameMarker, pubKFImage, pubKeyFrameList;
std::string world_frame_id, cam_frame_id, imu_frame_id, frameMap, frameBuildingComp, frameArchitecturalComp;
ros::Publisher pubTrackedMappoints, pubSegmentedPointcloud, pubPlanePointcloud, pubPlaneLabel, pubDoor,
    pubAllMappoints, pubFiducialMarker, pubRoom, pubFreespaceCluster;
ros::Time lastPlanePublishTime = ros::Time(0);

bool saveMapService(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    res.success = pSLAM->SaveMap(req.name);

    if (res.success)
        ROS_INFO("Map was saved as %s.osa", req.name.c_str());
    else
        ROS_ERROR("Map could not be saved.");

    return res.success;
}

bool saveMapPointsAsPCDService(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    res.success = pSLAM->SaveMapPointsAsPCD(req.name);

    if (res.success)
        ROS_INFO("Map points were saved as %s.pcd", req.name.c_str());
    else
        ROS_ERROR("Map points could not be saved.");

    return res.success;
}

bool saveTrajectoryService(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    const string cameraTrajectoryFile = req.name + "_cam_traj.txt";
    const string keyframeTrajectoryFile = req.name + "_kf_traj.txt";

    try
    {
        pSLAM->SaveTrajectoryEuRoC(cameraTrajectoryFile);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(keyframeTrajectoryFile);
        res.success = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        res.success = false;
    }
    catch (...)
    {
        std::cerr << "Unknows exeption" << std::endl;
        res.success = false;
    }

    if (!res.success)
        ROS_ERROR("Estimated trajectory could not be saved.");

    return res.success;
}

void setupServices(ros::NodeHandle &nodeHandler, std::string node_name)
{
    static ros::ServiceServer save_map_service = nodeHandler.advertiseService(node_name + "/save_map", saveMapService);
    static ros::ServiceServer save_traj_service = nodeHandler.advertiseService(node_name + "/save_traj", saveTrajectoryService);
    static ros::ServiceServer save_map_points_service = nodeHandler.advertiseService(node_name + "/save_map_points", saveMapPointsAsPCDService);
}

void setupPublishers(ros::NodeHandle &nodeHandler, image_transport::ImageTransport &image_transport, std::string node_name)
{
    // Basic
    pubTrackingImage = image_transport.advertise(node_name + "/tracking_image", 1);
    pubKeyFrameList = nodeHandler.advertise<nav_msgs::Path>(node_name + "/keyframe_list", 2);
    pubAllMappoints = nodeHandler.advertise<sensor_msgs::PointCloud2>(node_name + "/all_points", 1);
    pubCameraPose = nodeHandler.advertise<geometry_msgs::PoseStamped>(node_name + "/camera_pose", 1);
    pubKFImage = nodeHandler.advertise<segmenter_ros::VSGraphDataMsg>(node_name + "/keyframe_image", 50); // rate of keyframe generation is higher
    pubTrackedMappoints = nodeHandler.advertise<sensor_msgs::PointCloud2>(node_name + "/tracked_points", 1);
    pubKeyFrameMarker = nodeHandler.advertise<visualization_msgs::MarkerArray>(node_name + "/kf_markers", 1);
    pubFreespaceCluster = nodeHandler.advertise<sensor_msgs::PointCloud2>(node_name + "/freespace_clusters", 1);
    pubCameraPoseVis = nodeHandler.advertise<visualization_msgs::MarkerArray>(node_name + "/camera_pose_vis", 1);

    // Building Components
    pubDoor = nodeHandler.advertise<visualization_msgs::MarkerArray>(node_name + "/doors", 1);
    pubPlaneLabel = nodeHandler.advertise<visualization_msgs::MarkerArray>(node_name + "/plane_labels", 1);
    pubPlanePointcloud = nodeHandler.advertise<sensor_msgs::PointCloud2>(node_name + "/plane_point_clouds", 1);
    pubFiducialMarker = nodeHandler.advertise<visualization_msgs::MarkerArray>(node_name + "/fiducial_markers", 1);
    pubSegmentedPointcloud = nodeHandler.advertise<sensor_msgs::PointCloud2>(node_name + "/segmented_point_clouds", 1);

    // Structural Elements
    pubRoom = nodeHandler.advertise<visualization_msgs::MarkerArray>(node_name + "/rooms", 1);

    // Get body odometry if IMU data is also available
    if (sensorType == ORB_SLAM3::System::IMU_MONOCULAR || sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD)
        pubOdometry = nodeHandler.advertise<nav_msgs::Odometry>(node_name + "/body_odom", 1);

    // Showing planes using RViz Visual Tools
    visualTools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
        frameBuildingComp, "/plane_visuals", nodeHandler);
    visualTools->setAlpha(0.5);
    visualTools->loadMarkerPub();
    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();

    transformListener = std::make_shared<tf::TransformListener>();
}

void publishTopics(ros::Time msgTime, Eigen::Vector3f Wbb)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    // Avoid publishing NaN
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0))
        return;

    // Common topics
    publishCameraPose(Twc, msgTime);
    publishTFTransform(Twc, world_frame_id, cam_frame_id, msgTime);

    // Set a static transform between the world and map frame
    if (pubStaticTransform)
        publishStaticTFTransform(world_frame_id, frameMap, msgTime);

    // get the keyframes
    std::vector<ORB_SLAM3::KeyFrame *> keyframes = pSLAM->GetAllKeyFrames();

    // Setup publishers
    publishDoors(pSLAM->GetAllDoors(), msgTime);
    publishRooms(pSLAM->GetAllRooms(), msgTime);
    publishFiducialMarkers(pSLAM->GetAllMarkers(), msgTime);
    publishTrackingImage(pSLAM->GetCurrentFrame(), msgTime);
    publishKeyFrameImages(keyframes, msgTime);
    publishKeyFrameMarkers(keyframes, msgTime);

    // Publish pointclouds
    if (pubPointClouds)
    {
        publishAllPoints(pSLAM->GetAllMapPoints(), msgTime);
        publishSegmentedCloud(keyframes, msgTime);
        publishPlanes(pSLAM->GetAllPlanes(), msgTime);
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

        publishTFTransform(Twb, world_frame_id, imu_frame_id, msgTime);
        publishBodyOdometry(Twb, Vwb, Wwb, msgTime);
    }
}

void publishBodyOdometry(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, ros::Time msgTime)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id = imu_frame_id;
    odom_msg.header.frame_id = world_frame_id;
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

    pubOdometry.publish(odom_msg);
}

void publishCameraPose(Sophus::SE3f Tcw_SE3f, ros::Time msgTime)
{
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = cam_frame_id;
    poseMsg.header.stamp = msgTime;

    poseMsg.pose.position.x = Tcw_SE3f.translation().x();
    poseMsg.pose.position.y = Tcw_SE3f.translation().y();
    poseMsg.pose.position.z = Tcw_SE3f.translation().z();

    poseMsg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    poseMsg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    poseMsg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    poseMsg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pubCameraPose.publish(poseMsg);

    // Add a marker for visualization
    visualization_msgs::Marker cameraVisual;
    visualization_msgs::MarkerArray cameraVisualList;

    cameraVisual.id = 1;
    cameraVisual.color.a = 0.7;
    cameraVisual.scale.x = 0.5;
    cameraVisual.scale.y = 0.5;
    cameraVisual.scale.z = 0.5;
    cameraVisual.ns = "camera_pose";
    cameraVisual.header.stamp = msgTime;
    cameraVisual.action = cameraVisual.ADD;
    cameraVisual.lifetime = ros::Duration();
    cameraVisual.header.frame_id = world_frame_id;
    cameraVisual.mesh_use_embedded_materials = true;
    cameraVisual.type = visualization_msgs::Marker::MESH_RESOURCE;
    cameraVisual.mesh_resource =
        "package://orb_slam3_ros/config/Assets/camera.dae";

    cameraVisual.pose.position.x = Tcw_SE3f.translation().x();
    cameraVisual.pose.position.y = Tcw_SE3f.translation().y();
    cameraVisual.pose.position.z = Tcw_SE3f.translation().z();
    cameraVisual.pose.orientation.x = Tcw_SE3f.unit_quaternion().x();
    cameraVisual.pose.orientation.y = Tcw_SE3f.unit_quaternion().y();
    cameraVisual.pose.orientation.z = Tcw_SE3f.unit_quaternion().z();
    cameraVisual.pose.orientation.w = Tcw_SE3f.unit_quaternion().w();

    cameraVisualList.markers.push_back(cameraVisual);

    pubCameraPoseVis.publish(cameraVisualList);
}

void publishTFTransform(Sophus::SE3f T_SE3f, string frame_id, string child_frame_id, ros::Time msgTime)
{
    tf::Transform tf_transform = SE3fToTFTransform(T_SE3f);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, msgTime, frame_id, child_frame_id));
}

void publishStaticTFTransform(string parentFrameId, string childFrameId, ros::Time msgTime)
{
    // Variables
    tf2::Quaternion quat;
    geometry_msgs::TransformStamped transformStamped;
    static tf2_ros::StaticTransformBroadcaster broadcaster;

    // Set the values of the transform message
    transformStamped.header.stamp = msgTime;
    transformStamped.child_frame_id = childFrameId;
    transformStamped.header.frame_id = parentFrameId;

    // Set the translation of the transform to (0,0,0)
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    // Set the rotation of the transform to a fixed value
    quat.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    // Publish the static transform using the broadcaster
    broadcaster.sendTransform(transformStamped);
}

void publishFreeSpaceClusters(std::vector<std::vector<Eigen::Vector3d>> clusterPoints, ros::Time msgTime)
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
        if (colorIndex == fixedColors.size())
            colorIndex = 0;
    }

    // Check if the point cloud is empty
    if (freeSpaceCloud->empty())
        return;

    // Convert the point cloud to a PointCloud2 message
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*freeSpaceCloud, cloudMsg);

    // Set message header
    cloudMsg.header.stamp = msgTime;
    cloudMsg.header.frame_id = world_frame_id;

    // Publish the point cloud
    pubFreespaceCluster.publish(cloudMsg);
}

void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, ros::Time msgTime)
{
    // Check all keyframes and publish the ones that have not been published for Semantic Segmentation yet
    for (auto &keyframe : keyframe_vec)
    {
        if (keyframe->isPublished)
            continue;

        // Create an object of VSGraphDataMsg
        segmenter_ros::VSGraphDataMsg vsGraphPublisher = segmenter_ros::VSGraphDataMsg();

        std_msgs::Header header;
        header.stamp = msgTime;
        header.frame_id = world_frame_id;
        std_msgs::UInt64 kfId;
        kfId.data = keyframe->mnId;
        const sensor_msgs::ImagePtr rendered_image_msg =
            cv_bridge::CvImage(header, "bgr8", keyframe->mImage).toImageMsg();

        vsGraphPublisher.header = header;
        vsGraphPublisher.keyFrameId = kfId;
        vsGraphPublisher.keyFrameImage = *rendered_image_msg;

        pubKFImage.publish(vsGraphPublisher);
        keyframe->isPublished = true;
    }
}

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec)
{
    for (auto &keyframe : keyframe_vec)
        keyframe->clearClsClouds();
}

void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, ros::Time msgTime)
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
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregatedCloud, cloud_msg);

    // publish the pointcloud to be seen at the plane frame
    cloud_msg.header.frame_id = cam_frame_id;
    pubSegmentedPointcloud.publish(cloud_msg);
}

void publishTrackingImage(cv::Mat image, ros::Time msgTime)
{
    std_msgs::Header header;
    header.stamp = msgTime;
    header.frame_id = world_frame_id;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pubTrackingImage.publish(rendered_image_msg);
}

void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *> tracked_points, ros::Time msgTime)
{
    sensor_msgs::PointCloud2 cloud = mapPointToPointcloud(tracked_points, msgTime);
    pubTrackedMappoints.publish(cloud);
}

void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *> mapPoints, ros::Time msgTime)
{
    sensor_msgs::PointCloud2 cloud = mapPointToPointcloud(mapPoints, msgTime);
    pubAllMappoints.publish(cloud);
}

void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *> keyframe_vec, ros::Time msgTime)
{
    sort(keyframe_vec.begin(), keyframe_vec.end(), ORB_SLAM3::KeyFrame::lId);
    if (keyframe_vec.size() == 0)
        return;

    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker kf_markers;
    kf_markers.header.frame_id = world_frame_id;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = ros::Duration();
    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    visualization_msgs::Marker kf_lines;
    kf_lines.color.a = 0.15;
    kf_lines.color.r = 0.0;
    kf_lines.color.g = 0.0;
    kf_lines.color.b = 0.0;
    kf_lines.scale.x = 0.003;
    kf_lines.scale.y = 0.003;
    kf_lines.scale.z = 0.003;
    kf_lines.action = kf_lines.ADD;
    kf_lines.ns = "kf_lines";
    kf_lines.lifetime = ros::Duration();
    kf_lines.id = 1;
    kf_lines.header.stamp = ros::Time().now();
    kf_lines.header.frame_id = world_frame_id;
    kf_lines.type = visualization_msgs::Marker::LINE_LIST;

    nav_msgs::Path kf_list;
    kf_list.header.frame_id = world_frame_id;
    kf_list.header.stamp = msgTime;

    for (auto &keyframe : keyframe_vec)
    {
        geometry_msgs::Point kf_marker;

        Sophus::SE3f kf_pose = pSLAM->GetKeyFramePose(keyframe);
        kf_marker.x = kf_pose.translation().x();
        kf_marker.y = kf_pose.translation().y();
        kf_marker.z = kf_pose.translation().z();
        kf_markers.points.push_back(kf_marker);

        // populate the keyframe list
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(keyframe->mTimeStamp);
        pose.header.frame_id = world_frame_id;
        pose.pose.position.x = kf_pose.translation().x();
        pose.pose.position.y = kf_pose.translation().y();
        pose.pose.position.z = kf_pose.translation().z();
        pose.pose.orientation.w = kf_pose.unit_quaternion().w();
        pose.pose.orientation.x = kf_pose.unit_quaternion().x();
        pose.pose.orientation.y = kf_pose.unit_quaternion().y();
        pose.pose.orientation.z = kf_pose.unit_quaternion().z();
        kf_list.poses.push_back(pose);

        // // add lines to all keyframes in the covisibility graph
        // std::vector<ORB_SLAM3::KeyFrame *> covisibility = keyframe->GetBestCovisibilityKeyFrames(75);
        // // std::vector<ORB_SLAM3::KeyFrame *> covisibility = keyframe->GetVectorCovisibleKeyFrames();

        // for (auto &covis : covisibility)
        // {
        //     geometry_msgs::Point covis_marker;
        //     Sophus::SE3f covis_pose = pSLAM->GetKeyFramePose(covis);
        //     covis_marker.x = covis_pose.translation().x();
        //     covis_marker.y = covis_pose.translation().y();
        //     covis_marker.z = covis_pose.translation().z();
        //     kf_lines.points.push_back(kf_marker);
        //     kf_lines.points.push_back(covis_marker);
        // }

        // // get all planes from the keyframe
        // std::vector<ORB_SLAM3::Plane *> planes = keyframe->GetMapPlanes();

        // // attach lines to centroids of planes
        // for (auto &plane : planes)
        // {
        //     if (!plane)
        //         continue;
        //     // show only connections of planes with semantic types
        //     if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
        //         continue;

        //     // compute centroid of the plane from it's point cloud - point cloud already in world frame
        //     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = plane->getMapClouds();
        //     Eigen::Vector4f centroid;
        //     pcl::compute3DCentroid(*planeCloud, centroid);

        //     tf::Stamped<tf::Point> planePoint;
        //     planePoint.setX(centroid[0]);
        //     planePoint.setY(centroid[1]);
        //     planePoint.setZ(centroid[2]);
        //     planePoint.frame_id_ = frameBuildingComp;

        //     tf::Stamped<tf::Point> planePointTransformed;
        //     transform_listener->transformPoint(world_frame_id, ros::Time(0), planePoint,
        //                                        frameBuildingComp, planePointTransformed);

        //     geometry_msgs::Point point1;
        //     point1.x = planePointTransformed.x();
        //     point1.y = planePointTransformed.y();
        //     point1.z = planePointTransformed.z();
        //     kf_lines.points.push_back(point1);

        //     kf_lines.points.push_back(kf_marker);
        // }
    }
    markerArray.markers.push_back(kf_markers);
    // markerArray.markers.push_back(kf_lines);
    pubKeyFrameMarker.publish(markerArray);
    pubKeyFrameList.publish(kf_list);
}

void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *> markers, ros::Time msgTime)
{
    int numMarkers = markers.size();
    if (numMarkers == 0)
        return;

    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.resize(numMarkers);

    for (int idx = 0; idx < numMarkers; idx++)
    {
        visualization_msgs::Marker fiducial_marker;
        Sophus::SE3f markerPose = markers[idx]->getGlobalPose();

        fiducial_marker.color.a = 0;
        fiducial_marker.scale.x = 0.2;
        fiducial_marker.scale.y = 0.2;
        fiducial_marker.scale.z = 0.2;
        fiducial_marker.ns = "fiducial_markers";
        fiducial_marker.lifetime = ros::Duration();
        fiducial_marker.action = fiducial_marker.ADD;
        fiducial_marker.id = markerArray.markers.size();
        fiducial_marker.header.stamp = ros::Time().now();
        fiducial_marker.mesh_use_embedded_materials = true;
        fiducial_marker.header.frame_id = frameBuildingComp;
        fiducial_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        fiducial_marker.mesh_resource =
            "package://orb_slam3_ros/config/Assets/aruco_marker.dae";

        fiducial_marker.pose.position.x = markerPose.translation().x();
        fiducial_marker.pose.position.y = markerPose.translation().y();
        fiducial_marker.pose.position.z = markerPose.translation().z();
        fiducial_marker.pose.orientation.x = markerPose.unit_quaternion().x();
        fiducial_marker.pose.orientation.y = markerPose.unit_quaternion().y();
        fiducial_marker.pose.orientation.z = markerPose.unit_quaternion().z();
        fiducial_marker.pose.orientation.w = markerPose.unit_quaternion().w();

        markerArray.markers.push_back(fiducial_marker);
    }

    pubFiducialMarker.publish(markerArray);
}

void publishDoors(std::vector<ORB_SLAM3::Door *> doors, ros::Time msgTime)
{
    // If there are no doors, return
    int numDoors = doors.size();
    if (numDoors == 0)
        return;

    // Variables
    visualization_msgs::MarkerArray doorArray;
    doorArray.markers.resize(numDoors);

    for (int idx = 0; idx < numDoors; idx++)
    {
        Sophus::SE3f doorPose = doors[idx]->getGlobalPose();
        visualization_msgs::Marker door, doorLines, doorLabel;

        // Door values
        door.color.a = 0;
        door.ns = "doors";
        door.scale.x = 0.5;
        door.scale.y = 0.5;
        door.scale.z = 0.5;
        door.action = door.ADD;
        door.lifetime = ros::Duration();
        door.id = doorArray.markers.size();
        door.header.stamp = ros::Time().now();
        door.mesh_use_embedded_materials = true;
        door.header.frame_id = frameBuildingComp;
        door.type = visualization_msgs::Marker::MESH_RESOURCE;
        door.mesh_resource =
            "package://orb_slam3_ros/config/Assets/door.dae";

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
        doorLabel.lifetime = ros::Duration();
        doorLabel.text = doors[idx]->getName();
        doorLabel.id = doorArray.markers.size();
        doorLabel.header.stamp = ros::Time().now();
        doorLabel.header.frame_id = frameBuildingComp;
        doorLabel.pose.position.x = door.pose.position.x;
        doorLabel.pose.position.z = door.pose.position.z;
        doorLabel.pose.position.y = door.pose.position.y - 1.2;
        doorLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
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
        doorLines.lifetime = ros::Duration();
        doorLines.id = doorArray.markers.size();
        doorLines.header.stamp = ros::Time().now();
        doorLines.header.frame_id = frameBuildingComp;
        doorLines.type = visualization_msgs::Marker::LINE_LIST;

        geometry_msgs::Point point1;
        point1.x = doors[idx]->getMarker()->getGlobalPose().translation().x();
        point1.y = doors[idx]->getMarker()->getGlobalPose().translation().y();
        point1.z = doors[idx]->getMarker()->getGlobalPose().translation().z();
        doorLines.points.push_back(point1);

        geometry_msgs::Point point2;
        point2.x = rotatedDoorPose.translation().x();
        point2.y = rotatedDoorPose.translation().y();
        point2.z = rotatedDoorPose.translation().z();
        doorLines.points.push_back(point2);

        doorArray.markers.push_back(doorLines);
    }

    pubDoor.publish(doorArray);
}

void publishPlanes(std::vector<ORB_SLAM3::Plane *> planes, ros::Time msgTime)
{
    // Publish the planes, if any
    int numPlanes = planes.size();
    if (numPlanes == 0)
        return;

    // Check if sufficient time has passed since the last plane publication
    if ((msgTime - lastPlanePublishTime).toSec() < 3)
        return;
    lastPlanePublishTime = msgTime;

    // Variables
    visualization_msgs::MarkerArray planeLabelArray;
    planeLabelArray.markers.resize(numPlanes);
    visualization_msgs::Marker planeLabel, planeNormal;
    geometry_msgs::Point normalStartPoint, normalEndPoint;

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
        ORB_SLAM3::Plane::planeVariant planeType = plane->getPlaneType();
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
            continue;

        // Get the point clouds for the plane
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeClouds = plane->getMapClouds();
        if (planeClouds == nullptr || planeClouds->empty())
            continue;

        Eigen::Vector4d planeCoeffs = plane->getGlobalEquation().coeffs();
        for (const auto &point : planeClouds->points)
        {
            pcl::PointXYZRGB newPoint;
            newPoint.x = point.x;
            newPoint.y = point.y;
            newPoint.z = point.z;
            newPoint.r = point.r;
            newPoint.g = point.g;
            newPoint.b = point.b;

            // // Compute from plane equation - y for ground, z for wall
            // if (planeType == ORB_SLAM3::Plane::planeVariant::GROUND)
            //     newPoint.y = (-planeCoeffs(0) * point.x - planeCoeffs(2) * point.z - planeCoeffs(3)) / planeCoeffs(1);
            // else if (planeType == ORB_SLAM3::Plane::planeVariant::WALL)
            //     newPoint.z = (-planeCoeffs(0) * point.x - planeCoeffs(1) * point.y - planeCoeffs(3)) / planeCoeffs(2);

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
        planeLabel.color.r = color[0] / 255.0;
        planeLabel.color.g = color[1] / 255.0;
        planeLabel.color.b = color[2] / 255.0;
        planeLabel.lifetime = ros::Duration();
        planeLabel.pose.position.x = centroid.x();
        planeLabel.pose.position.z = centroid.z();
        planeLabel.pose.position.y = centroid.y() - 1.5;
        planeLabel.header.frame_id = frameBuildingComp;
        planeLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        // Create a marker for the plane normal (as an arrow)
        planeNormal.color.a = 1.0;
        planeNormal.scale.x = 0.01; // Shaft diameter
        planeNormal.scale.y = 0.05; // Arrowhead diameter
        planeNormal.scale.z = 0.05; // Arrowhead length
        planeNormal.ns = "planeNormal";
        planeNormal.header.stamp = msgTime;
        planeNormal.lifetime = ros::Duration();
        planeNormal.color.r = color[0] / 255.0;
        planeNormal.color.g = color[1] / 255.0;
        planeNormal.color.b = color[2] / 255.0;
        planeNormal.id = plane->getId() + numPlanes;
        planeNormal.header.frame_id = frameBuildingComp;
        planeNormal.type = visualization_msgs::Marker::ARROW;

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

    // convert the aggregated pointcloud to a pointcloud2 message
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregatedCloud, cloud_msg);

    // Set message header
    cloud_msg.header.stamp = msgTime;
    cloud_msg.header.frame_id = frameBuildingComp;

    // Publish the point cloud
    pubPlanePointcloud.publish(cloud_msg);
    pubPlaneLabel.publish(planeLabelArray);
}

void publishRooms(std::vector<ORB_SLAM3::Room *> rooms, ros::Time msgTime)
{
    // Publish rooms, if any
    int numRooms = rooms.size();
    if (numRooms == 0)
        return;

    // Visualization markers
    visualization_msgs::MarkerArray roomArray;
    roomArray.markers.resize(numRooms);

    for (int idx = 0; idx < numRooms; idx++)
    {
        // Variables
        string roomName = rooms[idx]->getName();
        string definedRoom = "package://orb_slam3_ros/config/Assets/room.dae";
        string undefinedRoom = "package://orb_slam3_ros/config/Assets/qmark.dae";
        string roomMesh = rooms[idx]->getHasKnownLabel() ? definedRoom : undefinedRoom;

        // Create color for room (orange for candidate, magenta for corridor, violet for normal room)
        std::vector<double>
            color = {1.0, 0.5, 0.0};
        if (rooms[idx]->getHasKnownLabel())
            if (rooms[idx]->getIsCorridor())
                color = {0.6, 0.0, 0.3};
            else
                color = {0.5, 0.1, 1.0};

        Eigen::Vector3d roomCenter = rooms[idx]->getRoomCenter();
        visualization_msgs::Marker room, roomWallLine, roomDoorLine, roomMarkerLine, roomLabel;

        // Room values
        room.ns = "rooms";
        room.scale.x = 0.6;
        room.scale.y = 0.6;
        room.scale.z = 0.6;
        room.color.a = 0.5;
        room.action = room.ADD;
        room.color.r = color[0];
        room.color.g = color[1];
        room.color.b = color[2];
        room.mesh_resource = roomMesh;
        room.lifetime = ros::Duration();
        room.id = roomArray.markers.size();
        room.header.stamp = ros::Time().now();
        room.mesh_use_embedded_materials = true;
        room.header.frame_id = frameArchitecturalComp;
        room.type = visualization_msgs::Marker::MESH_RESOURCE;

        // Rotation and displacement of the room for better visualization
        room.pose.orientation.x = 0.0;
        room.pose.orientation.y = 0.0;
        room.pose.orientation.z = 0.0;
        room.pose.orientation.w = 1.0;
        room.pose.position.x = roomCenter.x();
        room.pose.position.y = roomCenter.y();
        room.pose.position.z = roomCenter.z();
        roomArray.markers.push_back(room);

        // Room label (name)
        roomLabel.color.a = 1;
        roomLabel.color.r = 0;
        roomLabel.color.g = 0;
        roomLabel.color.b = 0;
        roomLabel.scale.z = 0.2;
        roomLabel.text = roomName;
        roomLabel.ns = "roomLabel";
        roomLabel.action = roomLabel.ADD;
        roomLabel.lifetime = ros::Duration();
        roomLabel.id = roomArray.markers.size();
        roomLabel.header.stamp = ros::Time().now();
        roomLabel.pose.position.x = roomCenter.x();
        roomLabel.pose.position.z = roomCenter.z();
        roomLabel.pose.position.y = roomCenter.y() - 0.7;
        roomLabel.header.frame_id = frameArchitecturalComp;
        roomLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        roomArray.markers.push_back(roomLabel);

        // Room to Plane (Wall) connection line
        roomWallLine.color.a = 0.5;
        roomWallLine.color.r = 0.0;
        roomWallLine.color.g = 0.0;
        roomWallLine.color.b = 0.0;
        roomWallLine.scale.x = 0.005;
        roomWallLine.scale.y = 0.005;
        roomWallLine.scale.z = 0.005;
        roomWallLine.ns = "room_wall_lines";
        roomWallLine.action = roomWallLine.ADD;
        roomWallLine.lifetime = ros::Duration();
        roomWallLine.id = roomArray.markers.size();
        roomWallLine.header.stamp = ros::Time().now();
        roomWallLine.header.frame_id = world_frame_id;
        roomWallLine.type = visualization_msgs::Marker::LINE_LIST;

        // Room to Door connection line
        roomDoorLine.color.a = 0.5;
        roomDoorLine.color.r = 0.0;
        roomDoorLine.color.g = 0.0;
        roomDoorLine.color.b = 0.0;
        roomDoorLine.scale.x = 0.005;
        roomDoorLine.scale.y = 0.005;
        roomDoorLine.scale.z = 0.005;
        roomDoorLine.ns = "room_door_lines";
        roomDoorLine.action = roomDoorLine.ADD;
        roomDoorLine.lifetime = ros::Duration();
        roomDoorLine.header.stamp = ros::Time().now();
        roomDoorLine.header.frame_id = world_frame_id;
        roomDoorLine.id = roomArray.markers.size() + 1;
        roomDoorLine.type = visualization_msgs::Marker::LINE_LIST;

        // Room to Marker connection line
        roomMarkerLine.color.a = 0.5;
        roomMarkerLine.color.r = 0.0;
        roomMarkerLine.color.g = 0.0;
        roomMarkerLine.color.b = 0.0;
        roomMarkerLine.scale.x = 0.005;
        roomMarkerLine.scale.y = 0.005;
        roomMarkerLine.scale.z = 0.005;
        roomMarkerLine.ns = "room_marker_lines";
        roomMarkerLine.lifetime = ros::Duration();
        roomMarkerLine.action = roomMarkerLine.ADD;
        roomMarkerLine.header.stamp = ros::Time().now();
        roomMarkerLine.header.frame_id = world_frame_id;
        roomMarkerLine.id = roomArray.markers.size() + 1;
        roomMarkerLine.type = visualization_msgs::Marker::LINE_LIST;

        // Get the room center in the world frame
        tf::Stamped<tf::Point> roomPoint, roomPointTransformed;

        roomPoint.setX(roomCenter.x());
        roomPoint.setY(roomCenter.y());
        roomPoint.setZ(roomCenter.z());
        roomPoint.frame_id_ = frameArchitecturalComp;
        transformListener->transformPoint(world_frame_id, ros::Time(0), roomPoint,
                                          frameArchitecturalComp, roomPointTransformed);

        // Room to Plane (Wall) connection line
        for (const auto wall : rooms[idx]->getWalls())
        {
            geometry_msgs::Point pointRoom, pointWall;
            tf::Stamped<tf::Point> pointWallInit, pointWallTransform;

            pointRoom.x = roomPointTransformed.x();
            pointRoom.y = roomPointTransformed.y();
            pointRoom.z = roomPointTransformed.z();
            roomWallLine.points.push_back(pointRoom);

            pointWallInit.frame_id_ = frameBuildingComp;
            pointWallInit.setX(wall->getCentroid().x());
            pointWallInit.setY(wall->getCentroid().y());
            pointWallInit.setZ(wall->getCentroid().z());
            transformListener->transformPoint(world_frame_id, ros::Time(0), pointWallInit,
                                              frameBuildingComp, pointWallTransform);

            pointWall.x = pointWallTransform.x();
            pointWall.y = pointWallTransform.y();
            pointWall.z = pointWallTransform.z();
            roomWallLine.points.push_back(pointWall);
        }

        // Room to Door connection line
        for (const auto door : rooms[idx]->getDoors())
        {
            geometry_msgs::Point pointRoom, pointDoor;
            tf::Stamped<tf::Point> pointDoorInit, pointDoorTransform;

            pointRoom.x = roomPointTransformed.x();
            pointRoom.y = roomPointTransformed.y();
            pointRoom.z = roomPointTransformed.z();
            roomDoorLine.points.push_back(pointRoom);

            pointDoorInit.frame_id_ = frameBuildingComp;
            pointDoorInit.setX(door->getGlobalPose().translation()(0));
            pointDoorInit.setY(door->getGlobalPose().translation()(1));
            pointDoorInit.setZ(door->getGlobalPose().translation()(2));
            transformListener->transformPoint(world_frame_id, ros::Time(0), pointDoorInit,
                                              frameBuildingComp, pointDoorTransform);

            pointDoor.x = pointDoorTransform.x();
            pointDoor.z = pointDoorTransform.z();
            pointDoor.y = pointDoorTransform.y() - 2.0;
            roomDoorLine.points.push_back(pointDoor);
        }

        // Room to Marker connection line
        geometry_msgs::Point pointRoom, pointMarker;
        tf::Stamped<tf::Point> pointMarkerInit, pointMarkerTransform;
        ORB_SLAM3::Marker *metaMarker = rooms[idx]->getMetaMarker();

        pointRoom.x = roomPointTransformed.x();
        pointRoom.y = roomPointTransformed.y();
        pointRoom.z = roomPointTransformed.z();
        roomMarkerLine.points.push_back(pointRoom);

        // In free space-based room candidate detection, the metaMarker is not available
        if (metaMarker != nullptr)
        {
            pointMarkerInit.frame_id_ = frameBuildingComp;
            pointMarkerInit.setX(metaMarker->getGlobalPose().translation()(0));
            pointMarkerInit.setY(metaMarker->getGlobalPose().translation()(1));
            pointMarkerInit.setZ(metaMarker->getGlobalPose().translation()(2));
            transformListener->transformPoint(world_frame_id, ros::Time(0), pointMarkerInit,
                                              frameBuildingComp, pointMarkerTransform);

            pointMarker.x = pointMarkerTransform.x();
            pointMarker.z = pointMarkerTransform.z();
            pointMarker.y = pointMarkerTransform.y();
            roomMarkerLine.points.push_back(pointMarker);
        }

        // Add items to the roomArray
        roomArray.markers.push_back(roomWallLine);
        roomArray.markers.push_back(roomDoorLine);
        roomArray.markers.push_back(roomMarkerLine);
    }

    pubRoom.publish(roomArray);
}

sensor_msgs::PointCloud2 mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints, ros::Time msgTime)
{
    // Variables
    const int numChannels = 3;
    sensor_msgs::PointCloud2 cloud;
    std::string channelId[] = {"x", "y", "z"};

    // Set the attributes of the point cloud
    cloud.header.stamp = msgTime;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = mapPoints.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = numChannels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(numChannels);

    // Set the fields of the point cloud
    for (int idx = 0; idx < numChannels; idx++)
    {
        cloud.fields[idx].count = 1;
        cloud.fields[idx].name = channelId[idx];
        cloud.fields[idx].offset = idx * sizeof(float);
        cloud.fields[idx].datatype = sensor_msgs::PointField::FLOAT32;
    }

    // Set the data of the point cloud
    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloudDataPtr = &(cloud.data[0]);

    // Populate the point cloud with the map points
    for (unsigned int idx = 0; idx < cloud.width; idx++)
        if (mapPoints[idx] && !mapPoints[idx]->isBad())
        {
            Eigen::Vector3d P3Dw = mapPoints[idx]->GetWorldPos().cast<double>();

            tf::Vector3 pointTranslation(P3Dw.x(), P3Dw.y(), P3Dw.z());

            float dataArray[numChannels] = {
                pointTranslation.x(),
                pointTranslation.y(),
                pointTranslation.z()};

            memcpy(cloudDataPtr + (idx * cloud.point_step), dataArray, numChannels * sizeof(float));
        }

    // Return the point cloud
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

tf::Transform SE3fToTFTransform(Sophus::SE3f data)
{
    Eigen::Matrix3f rotMatrix = data.rotationMatrix();
    Eigen::Vector3f transVector = data.translation();

    tf::Matrix3x3 rotationTF(
        rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2),
        rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2),
        rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2));

    tf::Vector3 translationTF(
        transVector(0),
        transVector(1),
        transVector(2));

    return tf::Transform(rotationTF, translationTF);
}

void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray)
{
    // The list of markers observed in the current frame
    std::vector<ORB_SLAM3::Marker *> currentMarkers;

    // Process the received marker array
    for (const auto &marker : markerArray.markers)
    {
        // Access information of each passed ArUco marker
        int markerId = marker.id;
        double visitTime = marker.header.stamp.toSec();
        geometry_msgs::Pose markerPose = marker.pose.pose;
        geometry_msgs::Point markerPosition = markerPose.position;            // (x,y,z)
        geometry_msgs::Quaternion markerOrientation = markerPose.orientation; // (x,y,z,w)

        Eigen::Vector3f markerTranslation(markerPosition.x, markerPosition.y, markerPosition.z);
        Eigen::Quaternionf markerQuaternion(markerOrientation.w, markerOrientation.x,
                                            markerOrientation.y, markerOrientation.z);
        Sophus::SE3f normalizedPose(markerQuaternion, markerTranslation);

        // Create a marker object of the currently visited marker
        ORB_SLAM3::Marker *currentMarker = new ORB_SLAM3::Marker();
        currentMarker->setOpId(-1);
        currentMarker->setId(markerId);
        currentMarker->setTime(visitTime);
        currentMarker->setMarkerInGMap(false);
        currentMarker->setLocalPose(normalizedPose);
        currentMarker->setMarkerType(ORB_SLAM3::Marker::markerVariant::UNKNOWN);

        // Add it to the list of observed markers
        currentMarkers.push_back(currentMarker);
    }

    // Add the new markers to the list of markers in buffer
    if (currentMarkers.size() > 0)
        markersBuffer.push_back(currentMarkers);
}

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

void setVoxbloxSkeletonCluster(const visualization_msgs::MarkerArray &skeletonArray)
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
                    geometry_msgs::PointStamped pointIn, pointOut;
                    pointIn.header.frame_id = frameMap;
                    pointIn.header.stamp = ros::Time(0);
                    pointIn.point = point;
                    transformListener->transformPoint(world_frame_id, pointIn, pointOut);

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