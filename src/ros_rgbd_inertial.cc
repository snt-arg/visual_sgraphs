#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    std::mutex mBufMutex;
    queue<sensor_msgs::ImuConstPtr> imuBuf;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb) : mpImuGb(pImuGb) {}

    void SyncWithImu();

    void GrabArUcoMarker(const aruco_msgs::MarkerArray &msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void GrabSegmentation(const segmenter_ros::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::MarkerArray &msgSkeletonGraph);
    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
                  const sensor_msgs::PointCloud2ConstPtr &msgPC);

    ImuGrabber *mpImuGb;
    std::mutex mBufMutex;
    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> imgPCBuf;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc > 1)
    {
        ROS_WARN("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle nodeHandler;
    image_transport::ImageTransport image_transport(nodeHandler);

    std::string voc_file, settings_file, sys_params_file;
    nodeHandler.param<std::string>(node_name + "/sys_params_file", sys_params_file, "file_not_set");
    nodeHandler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    nodeHandler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

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
    nodeHandler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    nodeHandler.param<double>(node_name + "/yaw", yaw, 0.0);
    nodeHandler.param<double>(node_name + "/roll", roll, 0.0);
    nodeHandler.param<double>(node_name + "/pitch", pitch, 0.0);

    nodeHandler.param<std::string>(node_name + "/frame_map", frameMap, "map");
    nodeHandler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");
    nodeHandler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    nodeHandler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    nodeHandler.param<bool>(node_name + "/static_transform", pubStaticTransform, false);
    nodeHandler.param<std::string>(node_name + "/frame_building_component", frameBC, "plane");
    nodeHandler.param<std::string>(node_name + "/frame_structural_element", frameSE, "room");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImuGrabber imugb;
    ImageGrabber igb(&imugb);
    sensorType = VS_GRAPHS::System::IMU_RGBD;

    pSLAM = new VS_GRAPHS::System(voc_file, settings_file, sys_params_file, sensorType, enable_pangolin);

    // Subscribe to get raw images and IMU data
    ros::Subscriber sub_imu = nodeHandler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    message_filters::Subscriber<sensor_msgs::Image> subImgRGB(nodeHandler, "/camera/rgb/image_raw", 500);
    message_filters::Subscriber<sensor_msgs::Image> subImgDepth(nodeHandler, "/camera/depth_registered/image_raw", 500);

    // Subscribe to get pointcloud from depth sensor
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointcloud(nodeHandler, "/camera/pointcloud", 500);

    // Synchronization of raw and depth images
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2>
        syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(500), subImgRGB, subImgDepth, subPointcloud);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2, _3));

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber sub_aruco = nodeHandler.subscribe("/aruco_marker_publisher/markers", 1,
                                                      &ImageGrabber::GrabArUcoMarker, &igb);

    // Subscriber for images obtained from the Semantic Segmentater
    ros::Subscriber sub_segmented_img = nodeHandler.subscribe("/camera/color/image_segment", 50,
                                                              &ImageGrabber::GrabSegmentation, &igb);

    // Subscriber to get the mesh from voxblox
    ros::Subscriber voxblox_skeleton_mesh = nodeHandler.subscribe("/voxblox_skeletonizer/sparse_graph", 1,
                                                                  &ImageGrabber::GrabVoxbloxSkeletonGraph, &igb);

    setupPublishers(nodeHandler, image_transport, node_name);
    setupServices(nodeHandler, node_name);

    // Syncing images with IMU
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD,
                            const sensor_msgs::PointCloud2ConstPtr &msgPC)
{
    mBufMutex.lock();

    if (!imgRGBBuf.empty())
        imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);

    if (!imgDBuf.empty())
        imgDBuf.pop();
    imgDBuf.push(msgD);

    if (!imgPCBuf.empty())
        imgPCBuf.pop();
    imgPCBuf.push(msgPC);

    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    while (1)
    {
        if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            cv::Mat im, depth;
            sensor_msgs::PointCloud2ConstPtr msgPC;
            double tIm = 0;

            tIm = imgRGBBuf.front()->header.stamp.toSec();
            if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutex.lock();
            ros::Time msg_time = imgRGBBuf.front()->header.stamp;
            im = GetImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            depth = GetImage(imgDBuf.front());
            imgDBuf.pop();
            msgPC = imgPCBuf.front();
            imgPCBuf.pop();

            this->mBufMutex.unlock();

            vector<VS_GRAPHS::IMU::Point> vImuMeas;
            vImuMeas.clear();
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(VS_GRAPHS::IMU::Point(acc, gyr, t));
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // Pointcloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // Convert pointclouds from ros to pcl format
            pcl::fromROSMsg(*msgPC, *cloud);

            // Find the marker with the minimum time difference compared to the current frame
            std::pair<double, std::vector<VS_GRAPHS::Marker *>> result = findNearestMarker(tIm);
            double minMarkerTimeDiff = result.first;
            std::vector<VS_GRAPHS::Marker *> matchedMarkers = result.second;

            // Tracking process sends markers found in this frame for tracking and clears the buffer
            if (minMarkerTimeDiff < 0.05)
            {
                pSLAM->TrackRGBD(im, depth, cloud, tIm, vImuMeas, "", matchedMarkers);
                markersBuffer.clear();
            }
            else
                pSLAM->TrackRGBD(im, depth, cloud, tIm, vImuMeas);

            publishTopics(msg_time, Wbb);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
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
        cv_imgSeg = cv_bridge::toCvCopy(msgSegImage.segmentedImage, sensor_msgs::image_encodings::BGR8);
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

void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::MarkerArray &msgSkeletonGraphs)
{
    // Pass the skeleton graph to a buffer to be processed by the SemanticSegmentation thread
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}