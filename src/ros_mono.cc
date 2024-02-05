#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    void GrabSegmentation(const sensor_msgs::ImageConstPtr &msgSegImage,
                          const sensor_msgs::PointCloud2ConstPtr &msgSegPrb);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc > 1)
        ROS_WARN("Arguments supplied via command line are ignored.");

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file, json_path;
    node_handler.param<std::string>(node_name + "/env_file", json_path, "");
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    if (json_path == "file_not_set")
    {
        ROS_ERROR("Please provide the JSON file containing environment data in the launch file!");
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<double>(node_name + "/yaw", yaw, 0.0);
    node_handler.param<double>(node_name + "/roll", roll, 0.0);
    node_handler.param<double>(node_name + "/pitch", pitch, 0.0);
    node_handler.param<double>(node_name + "/markers_impact", marker_impact, 1e10);
    node_handler.param<int>(node_name + "/pointclouds_threshold", pointcloud_size, 200);

    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/room_frame_id", room_frame_id, "room");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/struct_frame_id", struct_frame_id, "plane");
    node_handler.param<bool>(node_name + "/publish_static_transform", publish_static_transform, false);

    // Read environment data containing markers attached to rooms and corridors
    load_json_values(json_path);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImageGrabber igb;
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    // Set the environment data (doors) for the GeometricSegmentation thread
    pSLAM->setEnvDoors(env_doors);

    // Subscribe to get raw images
    ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber sub_aruco = node_handler.subscribe("/aruco_marker_publisher/markers",
                                                       1, &ImageGrabber::GrabArUcoMarker, &igb);

    // Synced subscriber for images obtained from the Semantic Segmentater
    message_filters::Subscriber<sensor_msgs::Image> sub_segmented_img(node_handler, "/camera/color/image_segment", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_segmented_prob(node_handler,
                                                                             "/camera/color/image_segment/probabilities", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> sync_seg;
    message_filters::Synchronizer<sync_seg> sync_segmenter(sync_seg(10), sub_segmented_img, sub_segmented_prob);
    sync_segmenter.registerCallback(boost::bind(&ImageGrabber::GrabSegmentation, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Find the marker with the minimum time difference compared to the current frame
    std::pair<double, std::vector<ORB_SLAM3::Marker *>> result = find_nearest_marker(cv_ptr->header.stamp.toSec());
    double min_time_diff = result.first;
    std::vector<ORB_SLAM3::Marker *> matched_markers = result.second;

    // Tracking process sends markers found in this frame for tracking and clears the buffer
    if (min_time_diff < 0.05)
    {
        Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec(), {},
                                                 "", matched_markers, env_doors, env_rooms);
        markers_buff.clear();
    }
    else
        Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    ros::Time msg_time = msg->header.stamp;
    publish_topics(msg_time);
}

void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &marker_array)
{
    // Pass the visited markers to a buffer to be processed later
    add_markers_to_buffer(marker_array);
}

void ImageGrabber::GrabSegmentation(const sensor_msgs::ImageConstPtr &msgSegImage,
                                    const sensor_msgs::PointCloud2ConstPtr &msgSegPrb)
{
    // [TODO] Add segmentation to the SLAM system
}