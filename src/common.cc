/**
 *
 * Common functions and variables across all modes (mono/stereo, with or w/o imu)
 *
 */

#include "common.h"

// Variables for ORB-SLAM3
ORB_SLAM3::System *pSLAM;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
int pointcloud_size = 200;
bool publish_static_transform;
double roll = 0, pitch = 0, yaw = 0;
image_transport::Publisher tracking_img_pub;
ros::Publisher pose_pub, odom_pub, kf_markers_pub;
rviz_visual_tools::RvizVisualToolsPtr wall_visual_tools;
std::shared_ptr<tf::TransformListener> transform_listener;
std::vector<std::vector<ORB_SLAM3::Marker *>> markers_buff;
std::string world_frame_id, cam_frame_id, imu_frame_id, map_frame_id, struct_frame_id, room_frame_id;
ros::Publisher tracked_mappoints_pub, all_mappoints_pub, fiducial_markers_pub, doors_pub, planes_pub, rooms_pub;

// List of semantic entities available in the real environment (filled using JSON)
std::vector<ORB_SLAM3::Room *> env_rooms;
std::vector<ORB_SLAM3::Door *> env_doors;

//////////////////////////////////////////////////
// Main functions
//////////////////////////////////////////////////

bool save_map_srv(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    res.success = pSLAM->SaveMap(req.name);

    if (res.success)
        ROS_INFO("Map was saved as %s.osa", req.name.c_str());
    else
        ROS_ERROR("Map could not be saved.");

    return res.success;
}

bool save_traj_srv(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    const string cam_traj_file = req.name + "_cam_traj.txt";
    const string kf_traj_file = req.name + "_kf_traj.txt";

    try
    {
        pSLAM->SaveTrajectoryEuRoC(cam_traj_file);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
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

void setup_services(ros::NodeHandle &node_handler, std::string node_name)
{
    static ros::ServiceServer save_map_service = node_handler.advertiseService(node_name + "/save_map", save_map_srv);
    static ros::ServiceServer save_traj_service = node_handler.advertiseService(node_name + "/save_traj", save_traj_srv);
}

void setup_publishers(ros::NodeHandle &node_handler, image_transport::ImageTransport &image_transport, std::string node_name)
{
    // Basic
    tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 1);
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>(node_name + "/camera_pose", 1);
    all_mappoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/all_points", 1);
    kf_markers_pub = node_handler.advertise<visualization_msgs::Marker>(node_name + "/kf_markers", 1000);
    tracked_mappoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/tracked_points", 1);

    // Semantic
    doors_pub = node_handler.advertise<visualization_msgs::MarkerArray>(node_name + "/doors", 1);
    rooms_pub = node_handler.advertise<visualization_msgs::MarkerArray>(node_name + "/rooms", 1);
    planes_pub = node_handler.advertise<visualization_msgs::MarkerArray>(node_name + "/planes", 1);
    fiducial_markers_pub = node_handler.advertise<visualization_msgs::MarkerArray>(node_name + "/fiducial_markers", 1);

    // Get body odometry if IMU data is also available
    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO ||
        sensor_type == ORB_SLAM3::System::IMU_RGBD)
        odom_pub = node_handler.advertise<nav_msgs::Odometry>(node_name + "/body_odom", 1);

    // Tools for showing planes
    wall_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
        struct_frame_id, "/rviz_wall_visual_tools");
    wall_visual_tools->loadMarkerPub();
    wall_visual_tools->deleteAllMarkers();
    wall_visual_tools->enableBatchPublishing();
    wall_visual_tools->setAlpha(0.5);

    transform_listener = std::make_shared<tf::TransformListener>();
}

void publish_topics(ros::Time msg_time, Eigen::Vector3f Wbb)
{
    // Setting parameters to be used in System.cc
    ORB_SLAM3::System::SystemParams params;
    params.pointCloudSize = pointcloud_size;
    pSLAM->SetSystemParameters(params);

    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    // Avoid publishing NaN
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0))
        return;

    // Common topics
    publish_camera_pose(Twc, msg_time);
    publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

    // If the boolean was set to True in the launch file, perform a transform
    if (publish_static_transform)
        publish_static_tf_transform(world_frame_id, map_frame_id, msg_time);

    // Setup publishers
    publish_doors(pSLAM->GetAllDoors(), msg_time);
    publish_rooms(pSLAM->GetAllRooms(), msg_time);
    publish_walls(pSLAM->GetAllPlanes(), msg_time);
    publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
    publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);
    publish_kf_markers(pSLAM->GetAllKeyframePoses(), msg_time);
    publish_fiducial_markers(pSLAM->GetAllMarkers(), msg_time);
    publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);

    // IMU-specific topics
    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO || sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        // Body pose and translational velocity can be obtained from ORB-SLAM3
        Sophus::SE3f Twb = pSLAM->GetImuTwb();
        Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

        // IMU provides body angular velocity in body frame (Wbb) which is transformed to world frame (Wwb)
        Sophus::Matrix3f Rwb = Twb.rotationMatrix();
        Eigen::Vector3f Wwb = Rwb * Wbb;

        publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
        if (publish_static_transform)
            publish_static_tf_transform(world_frame_id, map_frame_id, msg_time);
        publish_body_odom(Twb, Vwb, Wwb, msg_time);
    }
}

void publish_body_odom(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, ros::Time msg_time)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id = imu_frame_id;
    odom_msg.header.frame_id = world_frame_id;
    odom_msg.header.stamp = msg_time;

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

    odom_pub.publish(odom_msg);
}

void publish_camera_pose(Sophus::SE3f Tcw_SE3f, ros::Time msg_time)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id;
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Tcw_SE3f.translation().x();
    pose_msg.pose.position.y = Tcw_SE3f.translation().y();
    pose_msg.pose.position.z = Tcw_SE3f.translation().z();

    pose_msg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    pose_msg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    pose_msg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    pose_msg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pose_pub.publish(pose_msg);
}

void publish_tf_transform(Sophus::SE3f T_SE3f, string frame_id, string child_frame_id, ros::Time msg_time)
{
    tf::Transform tf_transform = SE3f_to_tfTransform(T_SE3f);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, msg_time, frame_id, child_frame_id));
}

/**
 * Publishes a static transformation (TF) between two frames
 *
 * @param frame_id The parent frame ID for the static transformation
 * @param child_frame_id The child frame ID for the static transformation
 * @param msg_time The timestamp for the transformation message
 */
void publish_static_tf_transform(string frame_id, string child_frame_id, ros::Time msg_time)
{
    // Create a static transform broadcaster
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // Create a transform stamped message
    geometry_msgs::TransformStamped static_stamped;

    // Set the values of the transform message
    static_stamped.header.stamp = msg_time;
    static_stamped.header.frame_id = frame_id;
    static_stamped.child_frame_id = child_frame_id;

    // Set the translation of the transform to (0,0,0)
    static_stamped.transform.translation.x = 0;
    static_stamped.transform.translation.y = 0;
    static_stamped.transform.translation.z = 0;

    // Set the rotation of the transform to a fixed value
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    static_stamped.transform.rotation.x = quat.x();
    static_stamped.transform.rotation.y = quat.y();
    static_stamped.transform.rotation.z = quat.z();
    static_stamped.transform.rotation.w = quat.w();

    // Publish the static transform using the broadcaster
    static_broadcaster.sendTransform(static_stamped);
}

void publish_tracking_img(cv::Mat image, ros::Time msg_time)
{
    std_msgs::Header header;
    header.stamp = msg_time;
    header.frame_id = world_frame_id;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    tracking_img_pub.publish(rendered_image_msg);
}

void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint *> tracked_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);
    tracked_mappoints_pub.publish(cloud);
}

void publish_all_points(std::vector<ORB_SLAM3::MapPoint *> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);
    all_mappoints_pub.publish(cloud);
}

// More details: http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html
void publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, ros::Time msg_time)
{
    int numKFs = vKFposes.size();
    if (numKFs == 0)
        return;

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

    for (int i = 0; i <= numKFs; i++)
    {
        geometry_msgs::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }

    kf_markers_pub.publish(kf_markers);
}

void publish_fiducial_markers(std::vector<ORB_SLAM3::Marker *> markers, ros::Time msg_time)
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
        fiducial_marker.header.frame_id = struct_frame_id;
        fiducial_marker.mesh_use_embedded_materials = true;
        fiducial_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        fiducial_marker.mesh_resource =
            "package://orb_slam3_ros/config/Visualization/aruco_marker.dae";

        fiducial_marker.pose.position.x = markerPose.translation().x();
        fiducial_marker.pose.position.y = markerPose.translation().y();
        fiducial_marker.pose.position.z = markerPose.translation().z();
        fiducial_marker.pose.orientation.x = markerPose.unit_quaternion().x();
        fiducial_marker.pose.orientation.y = markerPose.unit_quaternion().y();
        fiducial_marker.pose.orientation.z = markerPose.unit_quaternion().z();
        fiducial_marker.pose.orientation.w = markerPose.unit_quaternion().w();

        markerArray.markers.push_back(fiducial_marker);

        visualization_msgs::Marker fiducial_marker_lines;
        fiducial_marker_lines.color.a = 0.2;
        fiducial_marker_lines.color.r = 0.0;
        fiducial_marker_lines.color.g = 0.0;
        fiducial_marker_lines.color.b = 0.0;
        fiducial_marker_lines.scale.x = 0.005;
        fiducial_marker_lines.scale.y = 0.005;
        fiducial_marker_lines.scale.z = 0.005;
        fiducial_marker_lines.action = fiducial_marker.ADD;
        fiducial_marker_lines.ns = "fiducial_marker_lines";
        fiducial_marker_lines.lifetime = ros::Duration();
        fiducial_marker_lines.id = markerArray.markers.size();
        fiducial_marker_lines.header.stamp = ros::Time().now();
        fiducial_marker_lines.header.frame_id = world_frame_id;
        fiducial_marker_lines.type = visualization_msgs::Marker::LINE_LIST;

        const map<ORB_SLAM3::KeyFrame *, Sophus::SE3f> observations = markers[idx]->getObservations();
        for (map<ORB_SLAM3::KeyFrame *, Sophus::SE3f>::const_iterator obsId = observations.begin(), obLast = observations.end(); obsId != obLast; obsId++)
        {
            tf::Stamped<tf::Point> marker_point;
            marker_point.frame_id_ = struct_frame_id;
            marker_point.setX(markerPose.translation().x());
            marker_point.setY(markerPose.translation().y());
            marker_point.setZ(markerPose.translation().z());

            tf::Stamped<tf::Point> marker_point_transformed;
            transform_listener->transformPoint(world_frame_id, ros::Time(0), marker_point, struct_frame_id, marker_point_transformed);

            geometry_msgs::Point point1;
            point1.x = marker_point_transformed.x();
            point1.y = marker_point_transformed.y();
            point1.z = marker_point_transformed.z();
            fiducial_marker_lines.points.push_back(point1);

            ORB_SLAM3::KeyFrame *pKFi = obsId->first;
            tf::Stamped<tf::Point> keyframe_point;
            keyframe_point.setX(pKFi->GetPoseInverse().translation().x());
            keyframe_point.setY(pKFi->GetPoseInverse().translation().y());
            keyframe_point.setZ(pKFi->GetPoseInverse().translation().z());

            tf::Stamped<tf::Point> keyframe_point_transformed;
            geometry_msgs::Point point2;
            point2.x = pKFi->GetPoseInverse().translation().x();
            point2.y = pKFi->GetPoseInverse().translation().y();
            point2.z = pKFi->GetPoseInverse().translation().z();
            fiducial_marker_lines.points.push_back(point2);
        }
        markerArray.markers.push_back(fiducial_marker_lines);
    }

    fiducial_markers_pub.publish(markerArray);
}

void publish_doors(std::vector<ORB_SLAM3::Door *> doors, ros::Time msg_time)
{
    int numDoors = doors.size();
    if (numDoors == 0)
        return;

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
        door.header.frame_id = struct_frame_id;
        door.header.stamp = ros::Time().now();
        door.mesh_use_embedded_materials = true;
        door.type = visualization_msgs::Marker::MESH_RESOURCE;
        door.mesh_resource =
            "package://orb_slam3_ros/config/Visualization/door.dae";

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
        doorLabel.header.frame_id = struct_frame_id;
        doorLabel.header.stamp = ros::Time().now();
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
        doorLines.header.frame_id = struct_frame_id;
        doorLines.header.stamp = ros::Time().now();
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

    doors_pub.publish(doorArray);
}

void publish_walls(std::vector<ORB_SLAM3::Plane *> planes, ros::Time msg_time)
{
    // Publish the planes, if any
    int numPlanes = planes.size();
    if (numPlanes == 0)
        return;

    visualization_msgs::MarkerArray planeArray;
    planeArray.markers.resize(numPlanes);

    for (int idx = 0; idx < numPlanes; idx++)
    {
        // If the plane is not a wall, skip it
        if (planes[idx]->getPlaneType() != semanticType::WALL)
            continue;

        visualization_msgs::Marker plane, planePoints, planeLines;
        std::vector<double> color = planes[idx]->getColor();

        // Get the position of the planes from map-points to put it in the middle of the cluster
        Eigen::Vector3f centroid(0.0, 0.0, 0.0);
        // const pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapClouds = planes[idx]->getMapClouds();

        // for (const auto &mapCloud : mapClouds->points)
        // {
        //     // Plane plane
        //     Eigen::Vector3f mPosition(mapCloud.x, mapCloud.y, mapCloud.z);
        //     centroid += mPosition;
        //     // Plane rooms
        //     geometry_msgs::Point point;
        //     point.x = mPosition.x();
        //     point.y = mPosition.y();
        //     point.z = mPosition.z();
        //     planePoints.points.push_back(point);
        // }

        // // Calculate the centroid
        // if (mapClouds->points.size() > 0)
        //     centroid /= static_cast<float>(mapClouds->points.size());
        // planes[idx]->setCentroid(centroid);

        // Get the orientation of the plane
        if (planes[idx]->getMarkers().size() > 0)
        {
            // If the plane has markers, use the orientation of the marker
            Sophus::SE3f planeOrientation = planes[idx]->getMarkers().front()->getGlobalPose();

            // Plane values
            plane.ns = "planes";
            plane.scale.x = 0.5;
            plane.scale.y = 0.5;
            plane.scale.z = 0.5;
            plane.color.a = 0.5;
            plane.action = plane.ADD;
            plane.color.r = color[0] / 255;
            plane.color.g = color[1] / 255;
            plane.color.b = color[2] / 255;
            plane.lifetime = ros::Duration();
            plane.id = planeArray.markers.size();
            plane.header.stamp = ros::Time::now();
            plane.header.frame_id = struct_frame_id;
            plane.mesh_use_embedded_materials = true;
            plane.type = visualization_msgs::Marker::MESH_RESOURCE;
            plane.mesh_resource =
                "package://orb_slam3_ros/config/Visualization/plane.dae";

            // Rotation and displacement for better visualization
            planeOrientation *= Sophus::SE3f::rotX(-M_PI_2);

            plane.pose.position.x = centroid.x();
            plane.pose.position.y = centroid.y();
            plane.pose.position.z = centroid.z();
            plane.pose.orientation.x = planeOrientation.unit_quaternion().x();
            plane.pose.orientation.y = planeOrientation.unit_quaternion().y();
            plane.pose.orientation.z = planeOrientation.unit_quaternion().z();
            plane.pose.orientation.w = planeOrientation.unit_quaternion().w();
            planeArray.markers.push_back(plane);
        }
        else
        {
            // Otherwise, calculate the orientation of the plane using the map-points [TODO]
        }

        planePoints.color.a = 1;
        planePoints.scale.x = 0.03;
        planePoints.scale.y = 0.03;
        planePoints.scale.z = 0.03;
        planePoints.ns = "planes_points";
        planePoints.action = planePoints.ADD;
        planePoints.color.r = color[0] / 255;
        planePoints.color.g = color[1] / 255;
        planePoints.color.b = color[2] / 255;
        planePoints.lifetime = ros::Duration();
        planePoints.id = planeArray.markers.size();
        planePoints.header.stamp = ros::Time::now();
        planePoints.header.frame_id = struct_frame_id;
        planePoints.type = visualization_msgs::Marker::CUBE_LIST;
        planeArray.markers.push_back(planePoints);

        planeLines.color.a = 0.5;
        planeLines.color.r = 0.0;
        planeLines.color.g = 0.0;
        planeLines.color.b = 0.0;
        planeLines.scale.x = 0.005;
        planeLines.scale.y = 0.005;
        planeLines.scale.z = 0.005;
        planeLines.ns = "planeLines";
        planeLines.action = planeLines.ADD;
        planeLines.lifetime = ros::Duration();
        planeLines.id = planeArray.markers.size();
        planeLines.header.stamp = ros::Time().now();
        planeLines.header.frame_id = struct_frame_id;
        planeLines.type = visualization_msgs::Marker::LINE_LIST;

        for (const auto &planeMarker : planes[idx]->getMarkers())
        {
            geometry_msgs::Point point1;
            point1.x = planeMarker->getGlobalPose().translation().x();
            point1.y = planeMarker->getGlobalPose().translation().y();
            point1.z = planeMarker->getGlobalPose().translation().z();
            planeLines.points.push_back(point1);

            geometry_msgs::Point point2;
            point2.x = centroid.x();
            point2.y = centroid.y();
            point2.z = centroid.z();
            planeLines.points.push_back(point2);
        }
        planeArray.markers.push_back(planeLines);
    }

    planes_pub.publish(planeArray);
}

void publish_rooms(std::vector<ORB_SLAM3::Room *> rooms, ros::Time msg_time)
{
    int numRooms = rooms.size();
    if (numRooms == 0)
        return;

    visualization_msgs::MarkerArray roomArray;
    roomArray.markers.resize(numRooms);

    for (int idx = 0; idx < numRooms; idx++)
    {
        // Create color for room (magenta or violet based on room type)
        std::vector<double> color = {0.5, 0.1, 1.0};
        if (rooms[idx]->getWalls().size() == 2)
            color = {0.6, 0.0, 0.3};

        Eigen::Vector3d roomCenter = rooms[idx]->getRoomCenter();
        visualization_msgs::Marker room, roomWallLine, roomDoorLine, roomLabel;

        // Room values
        room.color.a = 0;
        room.ns = "rooms";
        room.scale.x = 0.6;
        room.scale.y = 0.6;
        room.scale.z = 0.6;
        room.color.a = 0.5;
        room.action = room.ADD;
        room.color.r = color[0];
        room.color.g = color[1];
        room.color.b = color[2];
        room.lifetime = ros::Duration();
        room.id = roomArray.markers.size();
        room.header.frame_id = room_frame_id;
        room.header.stamp = ros::Time().now();
        room.mesh_use_embedded_materials = true;
        room.type = visualization_msgs::Marker::MESH_RESOURCE;
        room.mesh_resource =
            "package://orb_slam3_ros/config/Visualization/room.dae";

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
        roomLabel.ns = "roomLabel";
        roomLabel.action = roomLabel.ADD;
        roomLabel.lifetime = ros::Duration();
        roomLabel.text = rooms[idx]->getName();
        roomLabel.id = roomArray.markers.size();
        roomLabel.header.frame_id = room_frame_id;
        roomLabel.header.stamp = ros::Time().now();
        roomLabel.pose.position.x = roomCenter.x();
        roomLabel.pose.position.z = roomCenter.z();
        roomLabel.pose.position.y = roomCenter.y() - 0.7;
        roomLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        roomArray.markers.push_back(roomLabel);

        // Room to plane connection line
        roomWallLine.color.a = 0.8;
        roomWallLine.scale.x = 0.01;
        roomWallLine.scale.y = 0.01;
        roomWallLine.scale.z = 0.01;
        roomWallLine.color.r = color[0];
        roomWallLine.color.g = color[1];
        roomWallLine.color.b = color[2];
        roomWallLine.ns = "room_wall_lines";
        roomWallLine.action = roomWallLine.ADD;
        roomWallLine.lifetime = ros::Duration();
        roomWallLine.id = roomArray.markers.size();
        roomWallLine.header.stamp = ros::Time().now();
        roomWallLine.header.frame_id = world_frame_id;
        roomWallLine.type = visualization_msgs::Marker::LINE_LIST;

        // Room to door connection line
        roomDoorLine.color.a = 0.8;
        roomDoorLine.scale.x = 0.01;
        roomDoorLine.scale.y = 0.01;
        roomDoorLine.scale.z = 0.01;
        roomDoorLine.color.r = color[0];
        roomDoorLine.color.g = color[1];
        roomDoorLine.color.b = color[2];
        roomDoorLine.ns = "room_door_lines";
        roomDoorLine.action = roomDoorLine.ADD;
        roomDoorLine.lifetime = ros::Duration();
        roomDoorLine.header.stamp = ros::Time().now();
        roomDoorLine.header.frame_id = world_frame_id;
        roomDoorLine.id = roomArray.markers.size() + 1;
        roomDoorLine.type = visualization_msgs::Marker::LINE_LIST;

        tf::Stamped<tf::Point> room_point;
        room_point.frame_id_ = room_frame_id;
        room_point.setX(roomCenter.x());
        room_point.setY(roomCenter.y());
        room_point.setZ(roomCenter.z());

        tf::Stamped<tf::Point> room_point_transformed;
        transform_listener->transformPoint(world_frame_id, ros::Time(0), room_point,
                                           room_frame_id, room_point_transformed);

        // Room to wall points connection line
        for (const auto wall : rooms[idx]->getWalls())
        {
            geometry_msgs::Point point1;
            point1.x = room_point_transformed.x();
            point1.y = room_point_transformed.y();
            point1.z = room_point_transformed.z();
            roomWallLine.points.push_back(point1);

            tf::Stamped<tf::Point> wall_point;
            wall_point.frame_id_ = struct_frame_id;
            wall_point.setX(wall->getCentroid().x());
            wall_point.setY(wall->getCentroid().y());
            wall_point.setZ(wall->getCentroid().z());
            tf::Stamped<tf::Point> wall_point_transformed;
            transform_listener->transformPoint(world_frame_id, ros::Time(0), wall_point,
                                               struct_frame_id, wall_point_transformed);

            geometry_msgs::Point point2;
            point2.x = wall_point_transformed.x();
            point2.y = wall_point_transformed.y();
            point2.z = wall_point_transformed.z();

            roomWallLine.points.push_back(point2);
        }

        // Room to door points connection line
        for (const auto door : rooms[idx]->getDoors())
        {
            geometry_msgs::Point point1;
            point1.x = room_point_transformed.x();
            point1.y = room_point_transformed.y();
            point1.z = room_point_transformed.z();
            roomDoorLine.points.push_back(point1);

            tf::Stamped<tf::Point> door_point;
            door_point.frame_id_ = struct_frame_id;
            tf::Stamped<tf::Point> door_point_transformed;
            door_point.setX(door->getGlobalPose().translation()(0));
            door_point.setY(door->getGlobalPose().translation()(1));
            door_point.setZ(door->getGlobalPose().translation()(2));
            transform_listener->transformPoint(world_frame_id, ros::Time(0), door_point,
                                               struct_frame_id, door_point_transformed);

            geometry_msgs::Point point2;
            point2.x = door_point_transformed.x();
            point2.z = door_point_transformed.z();
            point2.y = door_point_transformed.y() - 2.0;

            roomDoorLine.points.push_back(point2);
        }

        roomArray.markers.push_back(roomWallLine);
        roomArray.markers.push_back(roomDoorLine);
    }

    rooms_pub.publish(roomArray);
}

//////////////////////////////////////////////////
// Miscellaneous functions
//////////////////////////////////////////////////

sensor_msgs::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points, ros::Time msg_time)
{
    const int num_channels = 3; // x y z

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            tf::Vector3 point_translation(P3Dw.x(), P3Dw.y(), P3Dw.z());

            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z()};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}

cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f)
{
    cv::Mat T_cvmat;

    Eigen::Matrix4f T_Eig3f = T_SE3f.matrix();
    cv::eigen2cv(T_Eig3f, T_cvmat);

    return T_cvmat;
}

tf::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2));

    tf::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2));

    return tf::Transform(R_tf, t_tf);
}

//////////////////////////////////////////////////
// Fiducial Marker-related Modules
//////////////////////////////////////////////////

/**
 * Adds one/list of markers into a common buffer
 */
void add_markers_to_buffer(const aruco_msgs::MarkerArray &marker_array)
{
    // The list of markers observed in the current frame
    std::vector<ORB_SLAM3::Marker *> current_markers;

    // Process the received marker array
    for (const auto &marker : marker_array.markers)
    {
        // Access information of each passed ArUco marker
        int marker_id = marker.id;
        double visit_time = marker.header.stamp.toSec();
        geometry_msgs::Pose marker_pose = marker.pose.pose;
        geometry_msgs::Point marker_position = marker_pose.position;            // (x,y,z)
        geometry_msgs::Quaternion marker_orientation = marker_pose.orientation; // (x,y,z,w)

        Eigen::Vector3f marker_translation(marker_position.x, marker_position.y, marker_position.z);
        Eigen::Quaternionf marker_quaternion(marker_orientation.w, marker_orientation.x,
                                             marker_orientation.y, marker_orientation.z);
        Sophus::SE3f normalized_pose(marker_quaternion, marker_translation);

        // Create a marker object of the currently visited marker
        ORB_SLAM3::Marker *current_marker = new ORB_SLAM3::Marker();
        current_marker->setOpId(-1);
        current_marker->setId(marker_id);
        current_marker->setTime(visit_time);
        current_marker->setMarkerInGMap(false);
        current_marker->setLocalPose(normalized_pose);

        // Add it to the list of observed markers
        current_markers.push_back(current_marker);
    }

    // Add the new markers to the list of markers in buffer
    if (current_markers.size() > 0)
        markers_buff.push_back(current_markers);
}

/**
 * Processes the common marker buffer to get the one closest to the current marker
 */
std::pair<double, std::vector<ORB_SLAM3::Marker *>> find_nearest_marker(double frame_timestamp)
{
    double min_time_diff = 100;
    std::vector<ORB_SLAM3::Marker *> matched_markers;

    // Loop through the markers_buff
    for (const auto &markers : markers_buff)
    {
        double time_diff = markers[0]->getTime() - frame_timestamp;
        if (time_diff < min_time_diff)
        {
            min_time_diff = time_diff;
            matched_markers = markers;
        }
    }

    return std::make_pair(min_time_diff, matched_markers);
}

//////////////////////////////////////////////////
// Semantic Analysis Modules
//////////////////////////////////////////////////

/**
 * @brief Parses JSON values (database) and loads them into
 */
void load_json_values(string jsonFilePath)
{
    // Creating an object of the database loader
    ORB_SLAM3::DBParser parser;
    // Load JSON file
    json envData = parser.jsonParser(jsonFilePath);
    // Getting semantic entities
    env_rooms = parser.getEnvRooms(envData);
    env_doors = parser.getEnvDoors(envData);
}