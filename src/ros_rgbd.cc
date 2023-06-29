/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
 *
 */

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<double>(node_name + "/yaw", yaw, 0.0);
    node_handler.param<double>(node_name + "/roll", roll, 0.0);
    node_handler.param<double>(node_name + "/pitch", pitch, 0.0);
    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<bool>(node_name + "/publish_static_transform", publish_static_transform, false);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb;

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "/camera/depth_registered/image_raw", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

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

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Find the marker with the minimum time difference compared to the current frame
    std::pair<double, std::vector<ORB_SLAM3::Marker>> result = find_nearest_marker(cv_ptrRGB->header.stamp.toSec());
    double min_time_diff = result.first;
    std::vector<ORB_SLAM3::Marker> matched_markers = result.second;

    // Tracking process
    if (min_time_diff < 0.05)
    {
        Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec(),
                                            {}, "", matched_markers);
        aruco_marker_buff.clear();
    }
    else
    {
        Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    }

    ros::Time msg_time = cv_ptrRGB->header.stamp;

    publish_topics(msg_time);
}

void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &marker_array)
{
    // Pass the visited markers to a buffer to be processed later
    add_markers_to_buffer(marker_array);
}