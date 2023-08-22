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
bool publish_static_transform;
double roll = 0, pitch = 0, yaw = 0;
image_transport::Publisher tracking_img_pub;
rviz_visual_tools::RvizVisualToolsPtr wall_visual_tools;
ros::Publisher pose_pub, odom_pub, kf_markers_pub;
std::vector<std::vector<ORB_SLAM3::Marker *>> markers_buff;
std::string world_frame_id, cam_frame_id, imu_frame_id, map_frame_id, wall_frame_id;
ros::Publisher tracked_mappoints_pub, all_mappoints_pub, fiducial_markers_pub, doors_pub;

// List of semantic entities available in the real environment
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
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>(node_name + "/camera_pose", 1);

    tracked_mappoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/tracked_points", 1);

    all_mappoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/all_points", 1);

    tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 1);

    kf_markers_pub = node_handler.advertise<visualization_msgs::Marker>(node_name + "/kf_markers", 1000);

    fiducial_markers_pub = node_handler.advertise<visualization_msgs::MarkerArray>(node_name + "/fiducial_markers", 1);

    doors_pub = node_handler.advertise<visualization_msgs::MarkerArray>(node_name + "/doors", 1);

    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO || sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        odom_pub = node_handler.advertise<nav_msgs::Odometry>(node_name + "/body_odom", 1);
    }

    wall_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
        wall_frame_id, "/rviz_wall_visual_tools");
    wall_visual_tools->loadMarkerPub();
    wall_visual_tools->deleteAllMarkers();
    wall_visual_tools->enableBatchPublishing();
    wall_visual_tools->setAlpha(0.5);
}

void publish_topics(ros::Time msg_time, Eigen::Vector3f Wbb)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
        return;

    // Common topics
    publish_camera_pose(Twc, msg_time);
    publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

    // If the boolean was provided, perform a transform
    if (publish_static_transform)
        publish_static_tf_transform(world_frame_id, map_frame_id, msg_time);

    publish_doors(pSLAM->GetAllDoors(), msg_time);
    publish_walls(pSLAM->GetAllWalls(), msg_time);
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
 * Publishes a static transform between two coordinate frames
 */
void publish_static_tf_transform(string frame_id, string child_frame_id, ros::Time msg_time)
{
    // Create a static transform broadcaster
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // Create a transform stamped message
    geometry_msgs::TransformStamped static_stamped;

    // Set the timestamp of the transform message
    static_stamped.header.stamp = msg_time;
    // Set the parent frame ID for the transform
    static_stamped.header.frame_id = frame_id;
    // Set the child frame ID for the transform
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
        visualization_msgs::Marker marker;
        Sophus::SE3f markerPose = markers[idx]->getGlobalPose();

        marker.color.a = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.action = marker.ADD;
        marker.ns = "fiducial_markers";
        marker.lifetime = ros::Duration();
        marker.id = markerArray.markers.size();
        marker.header.stamp = ros::Time().now();
        marker.header.frame_id = wall_frame_id;
        marker.mesh_use_embedded_materials = true;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource =
            "package://orb_slam3_ros/config/Visualization/aruco_marker.dae";

        marker.pose.position.x = markerPose.translation().x();
        marker.pose.position.y = markerPose.translation().y();
        marker.pose.position.z = markerPose.translation().z();
        marker.pose.orientation.x = markerPose.unit_quaternion().x();
        marker.pose.orientation.y = markerPose.unit_quaternion().y();
        marker.pose.orientation.z = markerPose.unit_quaternion().z();
        marker.pose.orientation.w = markerPose.unit_quaternion().w();

        markerArray.markers.push_back(marker);
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
        visualization_msgs::Marker door;
        Sophus::SE3f doorPose = doors[idx]->getGlobalPose();

        door.color.a = 0;
        door.ns = "doors";
        door.scale.x = 0.5;
        door.scale.y = 0.5;
        door.scale.z = 0.5;
        door.action = door.ADD;
        door.lifetime = ros::Duration();
        door.id = doorArray.markers.size();
        door.header.stamp = ros::Time().now();
        door.header.frame_id = world_frame_id;
        door.mesh_use_embedded_materials = true;
        door.type = visualization_msgs::Marker::MESH_RESOURCE;
        door.mesh_resource =
            "package://orb_slam3_ros/config/Visualization/door.dae";

        // Rotation and displacement for better visualization
        Sophus::SE3f rotatedDoorPose = doorPose * Sophus::SE3f::rotX(-M_PI_2);
        rotatedDoorPose.translation().y() += 0.05;

        door.pose.position.x = rotatedDoorPose.translation().x();
        door.pose.position.y = rotatedDoorPose.translation().y();
        door.pose.position.z = rotatedDoorPose.translation().z();
        door.pose.orientation.x = rotatedDoorPose.unit_quaternion().x();
        door.pose.orientation.y = rotatedDoorPose.unit_quaternion().y();
        door.pose.orientation.z = rotatedDoorPose.unit_quaternion().z();
        door.pose.orientation.w = rotatedDoorPose.unit_quaternion().w();

        doorArray.markers.push_back(door);
    }

    doors_pub.publish(doorArray);
}

void publish_walls(std::vector<ORB_SLAM3::Wall *> walls, ros::Time msg_time) {
    int numWalls = walls.size();
    if (numWalls == 0)
        return;

    wall_visual_tools->deleteAllMarkers();
    for(const auto& wall : walls) {
        
        std::vector<MapPointStruct> mapPointsVec;
        for(const auto& mapPoint : wall->getMapPoints()) {
            MapPointStruct currentMapPoint(mapPoint->GetWorldPos());
            mapPointsVec.push_back(currentMapPoint);
        }

        std::vector<MapPointStruct> mapPointsVecRef = euclideanClustering(mapPointsVec);
        Eigen::Vector3f p_min, p_max;       
        float length = getMaxSegment(mapPointsVecRef, p_min, p_max);

        Eigen::Isometry3d plane_pose = compute_plane_pose(wall->getPlaneEquation(), p_min, p_max);
        double depth, width, height;

        if(fabs(wall->getPlaneEquation().coeffs()(0)) > fabs(wall->getPlaneEquation().coeffs()(1)) && 
            fabs(wall->getPlaneEquation().coeffs()(0)) > fabs(wall->getPlaneEquation().coeffs()(2))) {
            depth = rviz_visual_tools::SMALL_SCALE;
            width = fabs(p_min(2) - p_max(2));
            height = fabs(p_min(1) - p_max(1));
            wall_visual_tools->publishCuboid(plane_pose, depth, height, width, rviz_visual_tools::TRANSLUCENT);
        } else if(fabs(wall->getPlaneEquation().coeffs()(2)) > fabs(wall->getPlaneEquation().coeffs()(0)) && 
            fabs(wall->getPlaneEquation().coeffs()(2)) > fabs(wall->getPlaneEquation().coeffs()(1))) {
            depth = rviz_visual_tools::SMALL_SCALE;
            height = fabs(p_min(0) - p_max(0));
            width = fabs(p_min(1) - p_max(1));
            wall_visual_tools->publishCuboid(plane_pose, depth, height, width, rviz_visual_tools::TRANSLUCENT);
        }
        
    }

    wall_visual_tools->trigger();
}

std::vector<MapPointStruct> euclideanClustering(std::vector<MapPointStruct> points) 
{

    int cluster_id = 1;
    for (MapPointStruct& p1 : points) {
        if (p1.cluster_id != -1) {
            continue; // Skip points that are already assigned to a cluster
        }

        p1.cluster_id = cluster_id;

        for (MapPointStruct& p2 : points) {
            if (p2.cluster_id != -1) {
                continue;
            }

            double distance = (p1.coordinates - p2.coordinates).norm();

            if (distance <= 1.5) {
                p2.cluster_id = cluster_id;
            }
        }

        cluster_id++;
    }
        
    //group point with the same cluster id
    std::vector<std::vector<MapPointStruct>> cluster_points; 
    cluster_points.resize(cluster_id);
    for(MapPointStruct& p1 : points) {
        cluster_points[p1.cluster_id].push_back(p1);
    }
    
    int cluster_size =0;
    int seletected_cluster_id;
    for(int i =0; i< cluster_points.size(); ++i) {
        int current_cluster_size = cluster_points[i].size();
        if(current_cluster_size > cluster_size){
            cluster_size = current_cluster_size;
            seletected_cluster_id = i;
        } 
    }
    
    return cluster_points[seletected_cluster_id];
}

Eigen::Isometry3d compute_plane_pose(const g2o::Plane3D& planeEquation,
                                     Eigen::Vector3f& p_min,
                                     Eigen::Vector3f& p_max) {

    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d((p_min(0) - p_max(0)) / 2.0 + p_max(0),
                                        (p_min(1) - p_max(1)) / 2.0 + p_max(1),
                                        (p_min(2) - p_max(2)) / 2.0 + p_max(2));

    pose.linear() = planeEquation.rotation(planeEquation.coeffs().head<3>());
    return pose;
}

float getMaxSegment(const std::vector<MapPointStruct> mapPoints, 
                 Eigen::Vector3f &pmin, Eigen::Vector3f &pmax)
{
    float max_dist = std::numeric_limits<float>::min();
    int i_min = -1, i_max = -1; 

    for (size_t i = 0; i < mapPoints.size(); ++i)
    {
      for (size_t j = i; j < mapPoints.size(); ++j)
      {
        // Compute the distance 
        float dist = sqrt(pow(mapPoints[i].coordinates(0) - mapPoints[j].coordinates(0),2) + 
                        pow(mapPoints[i].coordinates(1) - mapPoints[j].coordinates(1),2) + 
                        pow(mapPoints[i].coordinates(2) - mapPoints[j].coordinates(2),2));

        if (dist <= max_dist)
          continue;

        max_dist = dist;
        i_min = i;
        i_max = j;
      }
    }

    if (i_min == -1 || i_max == -1)
      return (max_dist = std::numeric_limits<float>::min());

    pmin = mapPoints[i_min].coordinates;
    pmax = mapPoints[i_max].coordinates;
    return (std::sqrt (max_dist));
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