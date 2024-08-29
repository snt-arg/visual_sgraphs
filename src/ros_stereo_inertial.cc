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
    void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void GrabSegmentation(const segmenter_ros::SegmenterDataMsg &msgSegImage);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::MarkerArray &msgSkeletonGraph);

    ImuGrabber *mpImuGb;
    std::mutex mBufMutexLeft, mBufMutexRight;
    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Inertial");
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
    nodeHandler.param<bool>(node_name + "/publish_static_transform", pubStaticTransform, false);
    nodeHandler.param<std::string>(node_name + "/frame_building_comp", frameBuildingComp, "plane");
    nodeHandler.param<std::string>(node_name + "/frame_architectural_comp", frameArchitecturalComp, "room");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImuGrabber imugb;
    ImageGrabber igb(&imugb);
    sensorType = ORB_SLAM3::System::IMU_STEREO;

    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sys_params_file, sensorType, enable_pangolin);

    // Subscribe to get raw images and IMU data
    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    ros::Subscriber sub_imu = nodeHandler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = nodeHandler.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = nodeHandler.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber sub_aruco = nodeHandler.subscribe("/aruco_marker_publisher/markers", 1,
                                                      &ImageGrabber::GrabArUcoMarker, &igb);

    // Subscriber for images obtained from the Semantic Segmentater
    ros::Subscriber sub_segmented_img = nodeHandler.subscribe("/camera/color/image_segment", 10,
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

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            ros::Time msg_time = imgLeftBuf.front()->header.stamp;
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // Find the marker with the minimum time difference compared to the current frame
            std::pair<double, std::vector<ORB_SLAM3::Marker *>> result = findNearestMarker(tImLeft);
            double minMarkerTimeDiff = result.first;
            std::vector<ORB_SLAM3::Marker *> matchedMarkers = result.second;

            // Tracking process sends markers found in this frame for tracking and clears the buffer
            if (minMarkerTimeDiff < 0.05)
            {
                Sophus::SE3f Tcw = pSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas, "",
                                                      matchedMarkers);
                markersBuffer.clear();
            }
            else
            {
                Sophus::SE3f Tcw = pSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            }

            publishTopics(msg_time, Wbb);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
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