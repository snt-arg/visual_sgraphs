/**
 * 🚀 [vS-Graphs] Database Parse for JSON Files
 */

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber() {};

    void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    void GrabSegmentation(const segmenter_ros::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::MarkerArray &msgSkeletonGraph);
    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
                  const sensor_msgs::PointCloud2ConstPtr &msgPC);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc > 1)
        ROS_WARN("Arguments supplied via command line are ignored.");

    std::string nodeName = ros::this_node::getName();

    ros::NodeHandle nodeHandler;
    image_transport::ImageTransport imgTransport(nodeHandler);

    std::string vocFile, settingsFile, sysParamsFile;
    nodeHandler.param<std::string>(nodeName + "/voc_file", vocFile, "file_not_set");
    nodeHandler.param<std::string>(nodeName + "/settings_file", settingsFile, "file_not_set");
    nodeHandler.param<std::string>(nodeName + "/sys_params_file", sysParamsFile, "file_not_set");

    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        ROS_ERROR("[Error] 'vocabulary' and 'settings' are not provided in the launch file! Exiting...");
        ros::shutdown();
        return 1;
    }

    if (sysParamsFile == "file_not_set")
    {
        ROS_ERROR("[Error] The `YAML` file containing system parameters is not provided in the launch file! Exiting...");
        ros::shutdown();
        return 1;
    }

    bool enablePangolin;
    nodeHandler.param<bool>(nodeName + "/enable_pangolin", enablePangolin, true);

    nodeHandler.param<double>(nodeName + "/yaw", yaw, 0.0);
    nodeHandler.param<double>(nodeName + "/roll", roll, 0.0);
    nodeHandler.param<double>(nodeName + "/pitch", pitch, 0.0);

    nodeHandler.param<std::string>(nodeName + "/frame_map", frameMap, "map");
    nodeHandler.param<bool>(nodeName + "/colored_pointcloud", colorPointcloud, true);
    nodeHandler.param<bool>(nodeName + "/publish_pointclouds", pubPointClouds, true);
    nodeHandler.param<std::string>(nodeName + "/cam_frame_id", cam_frame_id, "camera");
    nodeHandler.param<std::string>(nodeName + "/world_frame_id", world_frame_id, "world");
    nodeHandler.param<bool>(nodeName + "/publish_static_transform", pubStaticTransform, false);
    nodeHandler.param<std::string>(nodeName + "/frame_building_comp", frameBuildingComp, "plane");
    nodeHandler.param<std::string>(nodeName + "/frame_architectural_comp", frameArchitecturalComp, "room");

    // Initializing system threads and getting ready to process frames
    ImageGrabber igb;
    sensorType = ORB_SLAM3::System::RGBD;

    pSLAM = new ORB_SLAM3::System(vocFile, settingsFile, sysParamsFile, sensorType, enablePangolin);

    // Subscribe to get raw images
    message_filters::Subscriber<sensor_msgs::Image> subImgRGB(nodeHandler, "/camera/rgb/image_raw", 500);
    message_filters::Subscriber<sensor_msgs::Image> subImgDepth(nodeHandler, "/camera/depth_registered/image_raw", 500);

    // Subscribe to get pointcloud from the depth sensor
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointcloud(nodeHandler, "/camera/depth/points", 500);

    // Synchronization of raw and depth images
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2>
        syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(500), subImgRGB, subImgDepth, subPointcloud);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2, _3));

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber subAruco = nodeHandler.subscribe("/aruco_marker_publisher/markers", 1,
                                                     &ImageGrabber::GrabArUcoMarker, &igb);

    // Subscriber for images obtained from the Semantic Segmentater
    ros::Subscriber subSegmentedImage = nodeHandler.subscribe("/camera/color/image_segment", 50,
                                                              &ImageGrabber::GrabSegmentation, &igb);

    // Subscriber to get the mesh from voxblox
    ros::Subscriber subVoxbloxSkeletonMesh = nodeHandler.subscribe("/voxblox_skeletonizer/sparse_graph", 1,
                                                                   &ImageGrabber::GrabVoxbloxSkeletonGraph, &igb);

    setupPublishers(nodeHandler, imgTransport, nodeName);
    setupServices(nodeHandler, nodeName);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
                            const sensor_msgs::PointCloud2ConstPtr &msgPC)
{
    // Variables
    cv_bridge::CvImageConstPtr cv_ptrD, cv_ptrRGB;

    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("[Error] Problem occured while running `cv_bridge`: %s", e.what());
        return;
    }

    // Convert pointclouds from ros to pcl format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msgPC, *cloud);

    // Find the marker with the minimum time difference compared to the current frame
    std::pair<double, std::vector<ORB_SLAM3::Marker *>>
        foundMarkerRes = findNearestMarker(cv_ptrRGB->header.stamp.toSec());
    double minMarkerTimeDiff = foundMarkerRes.first;
    std::vector<ORB_SLAM3::Marker *> matchedMarkers = foundMarkerRes.second;

    // Tracking process sends markers found in this frame for tracking and clears the buffer
    if (minMarkerTimeDiff < 0.05)
    {
        pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cloud,
                                            cv_ptrRGB->header.stamp.toSec(),
                                            {}, "", matchedMarkers);
        markersBuffer.clear();
    }
    else
    {
        pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cloud,
                                            cv_ptrRGB->header.stamp.toSec());
    }

    ros::Time msgTime = cv_ptrRGB->header.stamp;
    publishTopics(msgTime);
}

/**
 * @brief Callback function to get the markers detected by the `aruco_ros` library
 *
 * @param msgMarkerArray The markers detected by the `aruco_ros` library
 */
void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &msgMarkerArray)
{
    // Pass the visited markers to a buffer to be processed later
    addMarkersToBuffer(msgMarkerArray);
}

/**
 * @brief Callback function to get scene segmentation results from the SemanticSegmenter module
 *
 * @param msgSegImage The segmentation results from the SemanticSegmenter
 */
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

    // Convert to PCL PointCloud2 from `sensor_msgs` PointCloud2
    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmentedImageProbability, *pclPc2SegPrb);

    // Create the tuple to be appended to the segmentedImageBuffer
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> tuple(keyFrameId, cv_imgSeg->image, pclPc2SegPrb);

    // Add the segmented image to a buffer to be processed in the SemanticSegmentation thread
    pSLAM->addSegmentedImage(&tuple);
}

/**
 * @brief Callback function to get the skeleton graph from the `voxblox` module
 *
 * @param msgSkeletonGraphs The skeleton graph from the `voxblox` module
 */
void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::MarkerArray &msgSkeletonGraphs)
{
    // Pass the skeleton graph to a buffer to be processed by the SemanticSegmentation thread
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}