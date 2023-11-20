#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc > 1)
    {
        ROS_WARN("Arguments supplied via command line are ignored.");
    }

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
    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/plane_frame_id", plane_frame_id, "wall");
    node_handler.param<std::string>(node_name + "/room_frame_id", room_frame_id, "room");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<bool>(node_name + "/publish_static_transform", publish_static_transform, false);

    // Read environment data containing markers attached to rooms and corridors
    load_json_values(json_path);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImageGrabber igb;
    sensor_type = ORB_SLAM3::System::STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    // Subscribe to get raw images
    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(node_handler, "/camera/left/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(node_handler, "/camera/right/image_raw", 100);

    // Synchronization of stereo images
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber sub_aruco = node_handler.subscribe("/aruco_marker_publisher/markers", 1,
                                                       &ImageGrabber::GrabArUcoMarker, &igb);

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

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    ros::Time msg_time = msgLeft->header.stamp;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackStereo()
    Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msg_time.toSec());

    publish_topics(msg_time);
}

void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &marker_array)
{
    // Pass the visited markers to a buffer to be processed later
    add_markers_to_buffer(marker_array);
}