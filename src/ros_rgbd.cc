#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    void GrabSegmentation(const segmenter_ros::SegmenterDataMsg &msgSegImage);
    void GrabVoxBloxGraph(const visualization_msgs::MarkerArray &msgSkeletonGraph);
    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
                  const sensor_msgs::PointCloud2ConstPtr &msgPC);
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

    std::string voc_file, settings_file, sys_params_file;
    node_handler.param<std::string>(node_name + "/sys_params_file", sys_params_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    if (sys_params_file == "file_not_set")
    {
        ROS_ERROR("Please provide the YAML file containing system parameters in the launch file!");
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
    node_handler.param<std::string>(node_name + "/room_frame_id", room_frame_id, "room");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/struct_frame_id", struct_frame_id, "plane");
    node_handler.param<bool>(node_name + "/publish_static_transform", publish_static_transform, false);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImageGrabber igb;
    sensor_type = ORB_SLAM3::System::RGBD;

    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sys_params_file, sensor_type, enable_pangolin);

    // Subscribe to get raw images
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "/camera/depth_registered/image_raw", 100);

    // Subscribe to get pointcloud from depth sensor
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud(node_handler, "/camera/depth/points", 100);

    // Synchronization of raw and depth images
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2>
        sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img, sub_pointcloud);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2, _3));

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber sub_aruco = node_handler.subscribe("/aruco_marker_publisher/markers", 1,
                                                       &ImageGrabber::GrabArUcoMarker, &igb);

    // Subscriber for images obtained from the Semantic Segmentater
    ros::Subscriber sub_segmented_img = node_handler.subscribe("/camera/color/image_segment", 10,
                                                               &ImageGrabber::GrabSegmentation, &igb);

    // Subscriber to get the mesh from voxblox
    ros::Subscriber voxblox_skeleton_mesh = node_handler.subscribe("/voxblox_skeletonizer/sparse_graph", 1,
                                                                   &ImageGrabber::GrabVoxBloxGraph, &igb);

    setupPublishers(node_handler, image_transport, node_name);
    setupServices(node_handler, node_name);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
                            const sensor_msgs::PointCloud2ConstPtr &msgPC)
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

    // Pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert pointclouds from ros to pcl format
    pcl::fromROSMsg(*msgPC, *cloud);

    // Find the marker with the minimum time difference compared to the current frame
    std::pair<double, std::vector<ORB_SLAM3::Marker *>>
        result = findNearestMarker(cv_ptrRGB->header.stamp.toSec());
    double minMarkerTimeDiff = result.first;
    std::vector<ORB_SLAM3::Marker *> matchedMarkers = result.second;

    // Tracking process sends markers found in this frame for tracking and clears the buffer
    if (minMarkerTimeDiff < 0.05)
    {
        Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cloud,
                                            cv_ptrRGB->header.stamp.toSec(),
                                            {}, "", matchedMarkers);
        markersBuffer.clear();
    }
    else
    {
        Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cloud,
                                            cv_ptrRGB->header.stamp.toSec());
    }

    ros::Time msg_time = cv_ptrRGB->header.stamp;

    publishTopics(msg_time);
}

void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &markerArray)
{
    // Pass the visited markers to a buffer to be processed later
    addMarkersToBuffer(markerArray);
}

void ImageGrabber::GrabSegmentation(const segmenter_ros::SegmenterDataMsg &msgSegImage)
{
    // Fetch the segmentation results
    cv_bridge::CvImageConstPtr cv_imgSeg;
    uint64_t keyFrameId = msgSegImage.keyFrameId.data;

    try
    {
        cv_imgSeg = cv_bridge::toCvCopy(msgSegImage.segmentedImageUncertainty, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // convert to PCL PointCloud2 from sensor_msgs PointCloud2
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmentedImageProbability, *pclPc2SegPrb);

    // Create the tuple to be appended to the segmentedImageBuffer
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(keyFrameId, cv_imgSeg->image, pclPc2SegPrb);

    // Add the segmented image to a buffer to be processed in the SemanticSegmentation thread
    pSLAM->addSegmentedImage(&tuple);
}

void ImageGrabber::GrabVoxBloxGraph(const visualization_msgs::MarkerArray &msgSkeletonGraphs)
{
    // Pass the skeleton graph to a buffer to be processed by the SemanticSegmentation thread
    getVoxbloxSkeleton(msgSkeletonGraphs);
}