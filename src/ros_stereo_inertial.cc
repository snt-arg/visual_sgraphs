#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};

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
    void GrabSegmentation(const sensor_msgs::ImageConstPtr &msgSeg);

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
    node_handler.param<double>(node_name + "/markers_impact", marker_impact, 0.1);
    node_handler.param<int>(node_name + "/pointclouds_threshold", pointcloud_size, 200);

    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");
    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/room_frame_id", room_frame_id, "room");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/struct_frame_id", struct_frame_id, "plane");
    node_handler.param<bool>(node_name + "/publish_static_transform", publish_static_transform, false);

    // Read environment data containing markers attached to rooms and corridors
    load_json_values(json_path);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ImuGrabber imugb;
    ImageGrabber igb(&imugb);
    sensor_type = ORB_SLAM3::System::IMU_STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    // Set the environment data (doors) for the GeometricSegmentation thread
    pSLAM->setEnvDoors(env_doors);

    // Subscribe to get raw images and IMU data
    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = node_handler.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = node_handler.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

    // Subscribe to the markers detected by `aruco_ros` library
    ros::Subscriber sub_aruco = node_handler.subscribe("/aruco_marker_publisher/markers", 1,
                                                       &ImageGrabber::GrabArUcoMarker, &igb);

    // Subscribe to the segmentation image detected by `semantic_segmentation` library
    ros::Subscriber sub_segmentation = node_handler.subscribe("/camera/color/image_segment", 1,
                                                              &ImageGrabber::GrabSegmentation, &igb);

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    // Syncing images with IMU
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

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

            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f Tcw = pSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            publish_topics(msg_time, Wbb);

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

void ImageGrabber::GrabArUcoMarker(const aruco_msgs::MarkerArray &marker_array)
{
    // Pass the visited markers to a buffer to be processed later
    add_markers_to_buffer(marker_array);
}

void ImageGrabber::GrabSegmentation(const sensor_msgs::ImageConstPtr &msgSeg)
{
    // [TODO] Add segmentation to the SLAM system
}