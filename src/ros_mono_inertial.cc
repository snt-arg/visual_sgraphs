/**
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 */

#include "common.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <cmath>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

using namespace std;

namespace
{
constexpr double kMaxImuGap = 0.5;
constexpr double kMaxFrameImuDt = 0.1;
constexpr double kDefaultInstanceMaskTimeTolerance = 0.03;
constexpr double kMinOrbMaskAllowedRatio = 0.05;
constexpr float kMaxAccelNorm = 150.0f;
constexpr float kMaxGyroNorm = 50.0f;

bool isFiniteVector(const Eigen::Vector3f &v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Linearly interpolate a single accel sample to @target_time
*/
static ORB_SLAM3::IMU::Point interpolateAccel(
    const double target_time,
    const Eigen::Vector3f &cur_acc,  const double cur_t,
    const Eigen::Vector3f &prev_acc, const double prev_t,
    const Eigen::Vector3f &gyr)
{
    Eigen::Vector3f acc;

    if (prev_t == 0.0)
    {
        acc = cur_acc;
    }
    else if (target_time >= cur_t)
    {
        acc = cur_acc;
    }
    else if (target_time > prev_t)
    {
        const double factor = (target_time - prev_t) / (cur_t - prev_t);
        acc = prev_acc + static_cast<float>(factor) * (cur_acc - prev_acc);
    }
    else
    {
        acc = prev_acc;
    }

    return ORB_SLAM3::IMU::Point(
        acc.x(), acc.y(), acc.z(),
        gyr.x(), gyr.y(), gyr.z(),
        target_time);
}

// ---------------------------------------------------------------------------
// Shared IMU + Image state
// ---------------------------------------------------------------------------

struct SharedState
{
    std::mutex              mtx;
    std::condition_variable image_ready_cv;

    struct ImagePacket
    {
        cv::Mat image;
        double  timestamp = -1.0;
        cv::Mat instance_mask;
        double  instance_mask_time_diff = std::numeric_limits<double>::max();
        double  min_marker_time_diff = std::numeric_limits<double>::max();
        std::vector<ORB_SLAM3::Marker*> matched_markers;
    };

    struct AuxDepthFrame
    {
        rclcpp::Time stamp;
        double timestamp_sec = 0.0;
        std::string frame_id;
        sensor_msgs::msg::Image::ConstSharedPtr msg;
    };

    struct InstanceMaskFrame
    {
        rclcpp::Time stamp;
        double timestamp_sec = 0.0;
        std::string frame_id;
        cv::Mat mask;
    };

    // Raw gyro — collected at ~200 Hz
    std::deque<double>       gyro_timestamps;
    std::deque<Eigen::Vector3f>  gyro_data;

    // Accel interpolated to gyro timestamps
    std::deque<double>       accel_timestamps_sync;
    std::deque<Eigen::Vector3f>  accel_data_sync;

    // Running accel state for interpolation 
    Eigen::Vector3f  prev_accel_data;
    double       prev_accel_timestamp = 0.0;
    Eigen::Vector3f  cur_accel_data;
    double       cur_accel_timestamp  = 0.0;

    // Image queue keeps timestamps monotonic even if tracking briefly lags.
    std::queue<ImagePacket> image_queue;
    std::size_t             max_image_queue_size = 300;
    int                     dropped_frames = 0;

    std::deque<AuxDepthFrame> aux_depth_buffer;
    std::size_t              max_aux_depth_buffer_size = 120;

    std::deque<InstanceMaskFrame> instance_mask_buffer;
    std::size_t                   max_instance_mask_buffer_size = 300;
    double                        instance_mask_time_tolerance = kDefaultInstanceMaskTimeTolerance;

    // Marker state (populated inside the image callback path)
    double                          min_marker_time_diff = std::numeric_limits<double>::max();
    std::vector<ORB_SLAM3::Marker*> matched_markers;
};

// ---------------------------------------------------------------------------
// ImuGrabber — thin ROS2 subscriber node, pushes directly into SharedStat
// ---------------------------------------------------------------------------

class ImuGrabber : public rclcpp::Node
{
public:
    explicit ImuGrabber(std::shared_ptr<SharedState> state)
        : rclcpp::Node("imu_grabber", rclcpp::NodeOptions().use_global_arguments(false))
        , state_(std::move(state))
    {}

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

private:
    std::shared_ptr<SharedState> state_;
};

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    const double t = rclcpp::Time(msg->header.stamp).seconds();

    const Eigen::Vector3f acc(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
    const Eigen::Vector3f gyr(msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z);

    if (!std::isfinite(t) || !isFiniteVector(acc) || !isFiniteVector(gyr) ||
        acc.norm() > kMaxAccelNorm || gyr.norm() > kMaxGyroNorm)
    {
        RCLCPP_WARN(this->get_logger(), "Dropping invalid IMU sample at t=%.9f", t);
        return;
    }

    std::lock_guard<std::mutex> lock(state_->mtx);

    if (!state_->gyro_timestamps.empty())
    {
        const double last_t = state_->gyro_timestamps.back();
        if (t <= last_t)
        {
            RCLCPP_WARN(this->get_logger(), "Dropping non-monotonic IMU sample t=%.9f after %.9f", t, last_t);
            return;
        }
        if (t - last_t > kMaxImuGap)
        {
            RCLCPP_WARN(this->get_logger(), "Large IMU gap %.6f s; clearing stale IMU buffers", t - last_t);
            state_->gyro_timestamps.clear();
            state_->gyro_data.clear();
            state_->accel_timestamps_sync.clear();
            state_->accel_data_sync.clear();
            state_->prev_accel_data = Eigen::Vector3f::Zero();
            state_->cur_accel_data = Eigen::Vector3f::Zero();
            state_->prev_accel_timestamp = 0.0;
            state_->cur_accel_timestamp = 0.0;
        }
    }

    // --- Gyro path (high-rate, ~200 Hz) ---
    // Just buffer; accel will be interpolated to these timestamps
    state_->gyro_data.push_back(gyr);
    state_->gyro_timestamps.push_back(t);

    // --- Accel path (lower-rate, ~60 Hz or same rate depending on IMU) ---
    // ROS Imu message carries both in the same message, so we treat every
    // message as potentially carrying a new accel sample.  We detect a new
    // accel sample by checking if t > cur_accel_timestamp
    if (t > state_->cur_accel_timestamp)
    {
        state_->prev_accel_data      = state_->cur_accel_data;
        state_->prev_accel_timestamp = state_->cur_accel_timestamp;
        state_->cur_accel_data       = acc;
        state_->cur_accel_timestamp  = t;
    }
    // std::cout << "[ROS] Received IMU message at time " << t << std::endl;

    // Interpolate accel to cover any gyro timestamps not yet synced
    while (state_->gyro_timestamps.size() > state_->accel_timestamps_sync.size())
    {
        // std::cout << "[ROS] Interpolating accel for gyro timestamp " << state_->gyro_timestamps[state_->accel_timestamps_sync.size()] << std::endl;
        const int    idx         = static_cast<int>(state_->accel_timestamps_sync.size());
        const double target_time = state_->gyro_timestamps[idx];
        const Eigen::Vector3f &gyro_at_idx = state_->gyro_data[idx];

        ORB_SLAM3::IMU::Point pt = interpolateAccel(
            target_time,
            state_->cur_accel_data,  state_->cur_accel_timestamp,
            state_->prev_accel_data, state_->prev_accel_timestamp,
            gyro_at_idx);

        state_->accel_data_sync.push_back(pt.a);   // Eigen::Vector3f member
        state_->accel_timestamps_sync.push_back(target_time);
    }

    state_->image_ready_cv.notify_all();
}

// ---------------------------------------------------------------------------
// ImageGrabber — handles image callback and the main SyncWithImu loop
// ---------------------------------------------------------------------------

class ImageGrabber : public rclcpp::Node
{
public:
    struct AuxDepthOptions
    {
        bool use_aux_depth = true;
        bool encoding_is_metric = true;
        double time_tolerance = 0.05;
        float min_depth = 0.2f;
        float max_depth = 20.0f;
        int stride = 2;
        std::string scale_mode = "none";
    };

    struct SegmentOptions
    {
        std::string instance_mask_topic = "/camera/color/image_instance_masks";
        double instance_mask_time_tolerance = kDefaultInstanceMaskTimeTolerance;
        std::size_t max_instance_mask_buffer_size = 300;
    };

    ImageGrabber(
        std::shared_ptr<SharedState> state,
        AuxDepthOptions aux_depth_options,
        SegmentOptions segment_options)
        : rclcpp::Node("image_grabber", rclcpp::NodeOptions().use_global_arguments(false))
        , state_(std::move(state))
        , aux_depth_options_(std::move(aux_depth_options))
        , segment_options_(std::move(segment_options))
    {
        state_->max_instance_mask_buffer_size = segment_options_.max_instance_mask_buffer_size;
        state_->instance_mask_time_tolerance = segment_options_.instance_mask_time_tolerance;
    }

    ~ImageGrabber() = default;

    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void GrabAuxDepth(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void GrabInstanceMask(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msg);
    void GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msg);

    // Entry point for the sync thread
    void SyncWithImu();

    std::atomic<bool> mustStop{false};
private:
    bool FindClosestAuxDepth(double rgb_time, SharedState::AuxDepthFrame &depth_frame, double &dt_abs);
    bool ConvertAuxDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg, cv::Mat &depth_m);

    std::shared_ptr<SharedState>  state_;
    AuxDepthOptions aux_depth_options_;
    SegmentOptions segment_options_;
};

// ---------------------------------------------------------------------------
// GrabImage
// ---------------------------------------------------------------------------

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] cv_bridge exception: %s", e.what());
        return;
    }

    const double new_ts = rclcpp::Time(cv_ptr->header.stamp).seconds();

    // std::cout << "[ROS] Received image message at time " << new_ts << std::endl;

    // Resolve marker association for this timestamp.
    auto [min_diff, markers] = findNearestMarker(new_ts);

    std::unique_lock<std::mutex> lock(state_->mtx);

    if (!state_->image_queue.empty() && std::abs(state_->image_queue.back().timestamp - new_ts) < 1e-3)
        return;

    if (state_->image_queue.size() >= state_->max_image_queue_size)
    {
        state_->image_queue.pop();
        state_->dropped_frames++;
    }

    SharedState::ImagePacket packet;
    if (cv_ptr->image.channels() == 1 || cv_ptr->image.channels() == 3 || cv_ptr->image.channels() == 4)
    {
        packet.image = cv_ptr->image.clone();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[Error] Unsupported image encoding '%s' with %d channels.",
                     msg->encoding.c_str(), cv_ptr->image.channels());
        return;
    }
    packet.timestamp            = new_ts;
    packet.min_marker_time_diff = min_diff;
    packet.matched_markers      = std::move(markers);

    state_->image_queue.push(std::move(packet));
    state_->image_ready_cv.notify_all();
}

void ImageGrabber::GrabAuxDepth(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    if (!aux_depth_options_.use_aux_depth)
        return;

    SharedState::AuxDepthFrame frame;
    frame.stamp = rclcpp::Time(msg->header.stamp);
    frame.timestamp_sec = frame.stamp.seconds();
    frame.frame_id = msg->header.frame_id;
    frame.msg = msg;

    std::lock_guard<std::mutex> lock(state_->mtx);
    state_->aux_depth_buffer.push_back(std::move(frame));
    while (state_->aux_depth_buffer.size() > state_->max_aux_depth_buffer_size)
        state_->aux_depth_buffer.pop_front();
}

void ImageGrabber::GrabInstanceMask(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    if (!msg)
        return;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] Instance mask cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.type() != CV_16UC1 &&
        cv_ptr->image.type() != CV_32SC1 &&
        cv_ptr->image.type() != CV_8UC1)
    {
        RCLCPP_WARN(this->get_logger(), "Instance mask: unsupported encoding '%s'", msg->encoding.c_str());
        return;
    }

    SharedState::InstanceMaskFrame frame;
    frame.stamp = rclcpp::Time(msg->header.stamp);
    frame.timestamp_sec = frame.stamp.seconds();
    frame.frame_id = msg->header.frame_id;
    frame.mask = cv_ptr->image.clone();

    std::lock_guard<std::mutex> lock(state_->mtx);
    state_->instance_mask_buffer.push_back(std::move(frame));
    while (state_->instance_mask_buffer.size() > state_->max_instance_mask_buffer_size)
        state_->instance_mask_buffer.pop_front();
    state_->image_ready_cv.notify_all();
}

bool ImageGrabber::ConvertAuxDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg, cv::Mat &depth_m)
{
    if (!msg)
        return false;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] AuxDepth cv_bridge exception: %s", e.what());
        return false;
    }

    bool depth_is_metric = aux_depth_options_.encoding_is_metric;
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 || cv_ptr->image.type() == CV_32FC1)
    {
        depth_m = cv_ptr->image.clone();
    }
    else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || cv_ptr->image.type() == CV_16UC1)
    {
        cv_ptr->image.convertTo(depth_m, CV_32FC1, 0.001);
        depth_is_metric = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "AuxDepth: unsupported encoding '%s'", msg->encoding.c_str());
        return false;
    }

    for (int y = 0; y < depth_m.rows; ++y)
    {
        float *row = depth_m.ptr<float>(y);
        for (int x = 0; x < depth_m.cols; ++x)
        {
            const float d = row[x];
            if (!std::isfinite(d) ||
                (depth_is_metric && (d < aux_depth_options_.min_depth || d > aux_depth_options_.max_depth)))
                row[x] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    return true;
}

bool ImageGrabber::FindClosestAuxDepth(double rgb_time, SharedState::AuxDepthFrame &depth_frame, double &dt_abs)
{
    {
        std::lock_guard<std::mutex> lock(state_->mtx);

        if (!aux_depth_options_.use_aux_depth || state_->aux_depth_buffer.empty())
            return false;

        auto best_it = state_->aux_depth_buffer.end();
        dt_abs = std::numeric_limits<double>::max();

        for (auto it = state_->aux_depth_buffer.begin(); it != state_->aux_depth_buffer.end(); ++it)
        {
            const double dt = std::abs(rgb_time - it->timestamp_sec);
            if (dt < dt_abs)
            {
                dt_abs = dt;
                best_it = it;
            }
        }

        if (best_it == state_->aux_depth_buffer.end() || dt_abs > aux_depth_options_.time_tolerance)
            return false;

        depth_frame.stamp = best_it->stamp;
        depth_frame.timestamp_sec = best_it->timestamp_sec;
        depth_frame.frame_id = best_it->frame_id;
        depth_frame.msg = best_it->msg;
    }

    return true;
}

// ---------------------------------------------------------------------------
// SyncWithImu
// ---------------------------------------------------------------------------

void ImageGrabber::SyncWithImu()
{
    while (!mustStop && !pSLAM->isShutDown())
    {
        cv::Mat                              im;
        cv::Mat                              instanceMask;
        double                               tIm = 0.0;
        rclcpp::Time                         msgTime;
        Eigen::Vector3f                      Wbb = Eigen::Vector3f::Zero();
        std::vector<ORB_SLAM3::IMU::Point>   vImuMeas;
        double                               min_marker_diff;
        std::vector<ORB_SLAM3::Marker*>      matched_markers;
        std::size_t                          ready_imu_count = 0;

        // --- Critical section: wait, copy, clear ---
        {
            std::unique_lock<std::mutex> lk(state_->mtx);
            state_->image_ready_cv.wait(lk, [this] {
                return mustStop ||
                       (!state_->image_queue.empty() &&
                        !state_->instance_mask_buffer.empty() && 
                        !state_->gyro_timestamps.empty() &&
                        state_->gyro_timestamps.back() >= state_->image_queue.front().timestamp);
            });

            if (mustStop)
                break;

            // Log dropped frames
            if (state_->dropped_frames > 1)
                RCLCPP_WARN(this->get_logger(), "%d dropped frames", state_->dropped_frames - 1);
            state_->dropped_frames = 0;

            tIm     = state_->image_queue.front().timestamp;
            msgTime = rclcpp::Time(static_cast<int64_t>(tIm * 1e9));

            const std::size_t synced_imu_size =
                std::min(state_->gyro_timestamps.size(), state_->accel_data_sync.size());
            while (ready_imu_count < synced_imu_size &&
                   state_->gyro_timestamps[ready_imu_count] <= tIm)
            {
                ++ready_imu_count;
            }

            if (ready_imu_count < 2)
            {
                state_->image_queue.pop();
            }
            else
            {
                // Copy next queued image in timestamp order. Do this only after
                // the IMU buffer has reached this image timestamp.
                SharedState::ImagePacket packet = std::move(state_->image_queue.front());
                state_->image_queue.pop();

                im = std::move(packet.image);

                auto best_mask_it = state_->instance_mask_buffer.end();
                double best_mask_dt = std::numeric_limits<double>::max();
                while (!state_->instance_mask_buffer.empty() &&
                       state_->instance_mask_buffer.front().timestamp_sec <
                           tIm - state_->instance_mask_time_tolerance)
                {
                    state_->instance_mask_buffer.pop_front();
                }
                for (auto it = state_->instance_mask_buffer.begin();
                     it != state_->instance_mask_buffer.end(); ++it)
                {
                    const double dt = std::abs(tIm - it->timestamp_sec);
                    if (dt < best_mask_dt)
                    {
                        best_mask_dt = dt;
                        best_mask_it = it;
                    }
                }
                if (best_mask_it != state_->instance_mask_buffer.end() &&
                    best_mask_dt <= state_->instance_mask_time_tolerance)
                {
                    instanceMask = best_mask_it->mask.clone();
                    packet.instance_mask_time_diff = best_mask_dt;
                    state_->instance_mask_buffer.erase(
                        state_->instance_mask_buffer.begin(), std::next(best_mask_it));
                }

                // Copy marker state
                min_marker_diff  = packet.min_marker_time_diff;
                matched_markers  = std::move(packet.matched_markers);

                // Build IMU measurement vector from synced accel + gyro, consuming
                // only samples up to this image timestamp. Future IMU samples stay
                // buffered for the next frame, matching the ORB-SLAM3 ROS1 wrapper.
                for (std::size_t i = 0; i < ready_imu_count; ++i)
                {
                    const double imuTime = state_->gyro_timestamps.front();
                    const Eigen::Vector3f acc = state_->accel_data_sync.front();
                    const Eigen::Vector3f gyr = state_->gyro_data.front();

                    vImuMeas.emplace_back(
                        acc.x(), acc.y(), acc.z(),
                        gyr.x(), gyr.y(), gyr.z(),
                        imuTime);
                    Wbb = gyr;

                    state_->gyro_data.pop_front();
                    state_->gyro_timestamps.pop_front();
                    state_->accel_data_sync.pop_front();
                    state_->accel_timestamps_sync.pop_front();
                }
            }
        }
        // Lock released — TrackMonocular runs outside lock

        if (im.empty())
            continue;

        if (vImuMeas.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(),
                "Skipping frame t=%.9f because vImuMeas has only %zu samples",
                tIm, vImuMeas.size());
            continue;
        }
        if (vImuMeas.back().t > tIm)
        {
            RCLCPP_WARN(this->get_logger(),
                "Skipping frame t=%.9f because last IMU t=%.9f is newer than image",
                tIm, vImuMeas.back().t);
            continue;
        }

        bool invalidImuPacket = false;
        for (std::size_t i = 0; i < vImuMeas.size(); ++i)
        {
            const ORB_SLAM3::IMU::Point &imu = vImuMeas[i];
            if (!std::isfinite(imu.t) || !isFiniteVector(imu.a) || !isFiniteVector(imu.w) ||
                imu.a.norm() > kMaxAccelNorm || imu.w.norm() > kMaxGyroNorm)
            {
                invalidImuPacket = true;
                break;
            }

            if (i > 0)
            {
                const double dt = imu.t - vImuMeas[i - 1].t;
                if (dt <= 0.0 || dt > kMaxFrameImuDt)
                {
                    invalidImuPacket = true;
                    break;
                }
            }
        }
        if (invalidImuPacket)
        {
            RCLCPP_WARN(this->get_logger(), "Skipping frame t=%.9f because IMU packet is invalid", tIm);
            continue;
        }

        // Image scaling
        const float imageScale = pSLAM->GetImageScale();
        if (imageScale != 1.f)
        {
            const int w = static_cast<int>(im.cols * imageScale);
            const int h = static_cast<int>(im.rows * imageScale);
            cv::resize(im, im, cv::Size(w, h));
            if (!instanceMask.empty())
                cv::resize(instanceMask, instanceMask, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
        }

        // ORB extractor masks are CV_8UC1; non-zero pixels allow feature extraction.
        cv::Mat orbMask;
        if (!instanceMask.empty())
        {   
            cv::compare(instanceMask, 0, orbMask, cv::CMP_EQ);
            const double allowedRatio =
                static_cast<double>(cv::countNonZero(orbMask)) /
                static_cast<double>(orbMask.total());          
            if (allowedRatio < kMinOrbMaskAllowedRatio)
                orbMask.release();
        }

        // Track
        if (min_marker_diff < 0.05)
        {
            pSLAM->TrackMonocular(im, tIm, vImuMeas, "", matched_markers, orbMask);
            markersBuffer.clear();
        }
        else
        {
            pSLAM->TrackMonocular(im, tIm, vImuMeas, "", std::vector<ORB_SLAM3::Marker *>{}, orbMask);
        }
        publishTopics(msgTime, Wbb);
    }
}

void ImageGrabber::GrabSegmentation(const segmenter_ros::msg::SegmenterDataMsg &msgSegImage)
{
    cv_bridge::CvImageConstPtr cvImgSeg;
    const uint64_t keyFrameId = msgSegImage.key_frame_id.data;

    try
    {
        cvImgSeg = cv_bridge::toCvCopy(
            std::make_shared<sensor_msgs::msg::Image>(msgSegImage.segmented_image),
            sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[Error] cv_bridge exception: %s", e.what());
        return;
    }

    pcl::PCLPointCloud2::Ptr pclPc2SegPrb(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msgSegImage.segmented_image_probability, *pclPc2SegPrb);

    if (aux_depth_options_.use_aux_depth)
    {
        const double segTimestamp = rclcpp::Time(msgSegImage.header.stamp).seconds(); // Use msgSegImage timestamp (submsgs have empty timestamps)
        SharedState::AuxDepthFrame auxDepthFrame;
        double auxDepthDt = std::numeric_limits<double>::max();

        if (FindClosestAuxDepth(segTimestamp, auxDepthFrame, auxDepthDt))
        {
            cv::Mat auxDepthForSeg;
            if (ConvertAuxDepthImage(auxDepthFrame.msg, auxDepthForSeg))
            {
                if (!auxDepthForSeg.empty() && auxDepthForSeg.size() != cvImgSeg->image.size())
                {
                    cv::Mat resizedDepth;
                    cv::resize(auxDepthForSeg, resizedDepth, cvImgSeg->image.size(), 0, 0, cv::INTER_NEAREST);
                    auxDepthForSeg = resizedDepth;
                }

                RCLCPP_INFO(this->get_logger(), "AuxDepth: matched depth dt = %.6f for segmented keyframe ID %lu",
                            auxDepthDt, static_cast<unsigned long>(keyFrameId));
                pSLAM->AttachAuxDepthToKeyFrame(keyFrameId, auxDepthForSeg,
                                                auxDepthFrame.timestamp_sec, auxDepthFrame.frame_id,
                                                aux_depth_options_.min_depth, aux_depth_options_.max_depth,
                                                aux_depth_options_.stride, aux_depth_options_.scale_mode);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "AuxDepth: no depth found for segmented frame timestamp %.9f keyframe ID %lu",
                        segTimestamp, static_cast<unsigned long>(keyFrameId));
        }
    }

    auto tuple = std::make_tuple(keyFrameId, cvImgSeg->image, pclPc2SegPrb);
    pSLAM->addSegmentedImage(&tuple);
}

void ImageGrabber::GrabVoxbloxSkeletonGraph(const visualization_msgs::msg::MarkerArray &msgSkeletonGraphs)
{
    setVoxbloxSkeletonCluster(msgSkeletonGraphs);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vs_graphs");

    if (argc > 1)
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");

    // --- Parameters ---
    node->declare_parameter<double>("yaw",   0.0);
    node->declare_parameter<double>("roll",  0.0); 
    node->declare_parameter<double>("pitch", 0.0);
    node->declare_parameter<bool>("enable_pangolin",    true);
    node->declare_parameter<bool>("static_transform",   false);
    node->declare_parameter<bool>("colored_pointcloud", true);
    node->declare_parameter<bool>("publish_pointclouds", true);
    node->declare_parameter<std::string>("frame_imu",                "imu");
    node->declare_parameter<std::string>("frame_map",                "map");
    node->declare_parameter<std::string>("frame_world",              "world");
    node->declare_parameter<std::string>("frame_camera",             "camera");
    node->declare_parameter<std::string>("frame_structural_element", "struc_elem");
    node->declare_parameter<std::string>("frame_building_component", "build_comp");
    node->declare_parameter<std::string>("voc_file",         "file_not_set");
    node->declare_parameter<std::string>("settings_file",    "file_not_set");
    node->declare_parameter<std::string>("sys_params_file",  "file_not_set");
    node->declare_parameter<bool>("use_aux_depth", true);
    node->declare_parameter<std::string>("aux_depth_topic", "/camera/depth_da3/image_rect");
    node->declare_parameter<bool>("aux_depth_encoding_is_metric", true);
    node->declare_parameter<double>("aux_depth_time_tolerance", 0.05);
    node->declare_parameter<double>("aux_depth_min", 0.2);
    node->declare_parameter<double>("aux_depth_max", 20.0);
    node->declare_parameter<int>("aux_depth_stride", 2);
    node->declare_parameter<int>("aux_depth_buffer_size", 600);
    node->declare_parameter<std::string>("aux_depth_scale_mode", "none");
    node->declare_parameter<std::string>("instance_mask_topic", "/camera/color/image_instance_masks");
    node->declare_parameter<double>("instance_mask_time_tolerance", kDefaultInstanceMaskTimeTolerance);
    node->declare_parameter<int>("instance_mask_buffer_size", 300);

    const std::string vocFile      = node->get_parameter("voc_file").as_string();
    const std::string settingsFile = node->get_parameter("settings_file").as_string();
    const std::string sysParamsFile = node->get_parameter("sys_params_file").as_string();
    const std::string auxDepthTopic = node->get_parameter("aux_depth_topic").as_string();
    ImageGrabber::AuxDepthOptions auxDepthOptions;
    auxDepthOptions.use_aux_depth = node->get_parameter("use_aux_depth").as_bool();
    auxDepthOptions.encoding_is_metric = node->get_parameter("aux_depth_encoding_is_metric").as_bool();
    auxDepthOptions.time_tolerance = node->get_parameter("aux_depth_time_tolerance").as_double();
    auxDepthOptions.min_depth = static_cast<float>(node->get_parameter("aux_depth_min").as_double());
    auxDepthOptions.max_depth = static_cast<float>(node->get_parameter("aux_depth_max").as_double());
    auxDepthOptions.stride = std::max(1, static_cast<int>(node->get_parameter("aux_depth_stride").as_int()));
    auxDepthOptions.scale_mode = node->get_parameter("aux_depth_scale_mode").as_string();

    ImageGrabber::SegmentOptions segmentOptions;
    segmentOptions.instance_mask_topic = node->get_parameter("instance_mask_topic").as_string();
    segmentOptions.instance_mask_time_tolerance =
        std::max(0.0, node->get_parameter("instance_mask_time_tolerance").as_double());
    segmentOptions.max_instance_mask_buffer_size =
        static_cast<std::size_t>(std::max(1, static_cast<int>(node->get_parameter("instance_mask_buffer_size").as_int())));
    if (auxDepthOptions.scale_mode != "none" && auxDepthOptions.scale_mode != "map_median")
    {
        RCLCPP_WARN(node->get_logger(), "AuxDepth: unsupported aux_depth_scale_mode '%s', using 'none'",
                    auxDepthOptions.scale_mode.c_str());
        auxDepthOptions.scale_mode = "none";
    }
    if (auxDepthOptions.use_aux_depth && !auxDepthOptions.encoding_is_metric && auxDepthOptions.scale_mode == "none")
    {
        RCLCPP_WARN(node->get_logger(),
                    "AuxDepth: aux_depth_encoding_is_metric is false with scale_mode 'none'; depth will be interpreted as meters by segmentation");
    }
    RCLCPP_INFO(node->get_logger(), "AuxDepth: use_aux_depth = %s",
                auxDepthOptions.use_aux_depth ? "true" : "false");

    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "[Error] 'voc_file' and 'settings_file' not set. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    if (sysParamsFile == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "[Error] 'sys_params_file' not set. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
 
    // Populate globals required by common.h helpers (publishTopics, etc.)
    yaw              = node->get_parameter("yaw").as_double();
    roll             = node->get_parameter("roll").as_double();
    pitch            = node->get_parameter("pitch").as_double();
    frameImu         = node->get_parameter("frame_imu").as_string();
    frameMap         = node->get_parameter("frame_map").as_string();
    frameWorld       = node->get_parameter("frame_world").as_string();
    frameCamera      = node->get_parameter("frame_camera").as_string();
    colorPointcloud  = node->get_parameter("colored_pointcloud").as_bool();
    pubPointClouds   = node->get_parameter("publish_pointclouds").as_bool();
    frameBC          = node->get_parameter("frame_building_component").as_string();
    frameSE          = node->get_parameter("frame_structural_element").as_string();
    pubStaticTransform = node->get_parameter("static_transform").as_bool();
    const bool enablePangolin = node->get_parameter("enable_pangolin").as_bool();

    // --- SLAM system ---
    sensorType = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(vocFile, settingsFile, sysParamsFile, sensorType, enablePangolin);

    // --- Shared state ---
    auto state = std::make_shared<SharedState>();
    state->max_aux_depth_buffer_size =
        static_cast<std::size_t>(std::max(1, static_cast<int>(node->get_parameter("aux_depth_buffer_size").as_int())));

    // --- TF broadcasters (one pair, shared) ---
    tfBroadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(node);
    staticTfBroadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // --- Grabber nodes ---
    auto imugb = std::make_shared<ImuGrabber>(state);
    auto igb   = std::make_shared<ImageGrabber>(state, auxDepthOptions, segmentOptions);

    // --- QoS (sensor data profile) ---
    // rmw_qos_profile_sensor_data defaults to a keep-last depth of 5, which at ~400Hz is only a
    // ~12.5ms buffer -- any delay servicing the IMU callback longer than that (e.g. the executor
    // busy on a slower vision callback, see imuCallbackGroup below) causes DDS to silently drop
    // the backlog before GrabImu ever runs, starving SyncWithImu's ready_imu_count even though
    // the true publish rate is far higher than the ~30Hz camera rate would suggest is necessary.
    // Widened to 0.5s worth of samples to absorb realistic scheduling jitter/bursts.
    rclcpp::QoS sensorQos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensorQos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    sensorQos.durability(rclcpp::DurabilityPolicy::Volatile);
    sensorQos.keep_last(200);

    using sensor_msgs::msg::Image;
    using sensor_msgs::msg::Imu;

    // automatically_add_to_executor_with_node = false: this group is serviced by its own
    // dedicated executor/thread below instead of the shared 4-thread pool, so it must not also
    // be picked up by executor.add_node(node) below (a callback group can only belong to one
    // executor at a time).
    auto imuCallbackGroup = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, /*automatically_add_to_executor_with_node=*/false);
    auto imageCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto auxDepthCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto instanceMaskCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto semanticCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto voxbloxCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions imuOptions;
    imuOptions.callback_group = imuCallbackGroup;
    rclcpp::SubscriptionOptions imageOptions;
    imageOptions.callback_group = imageCallbackGroup;
    rclcpp::SubscriptionOptions auxDepthOptionsSub;
    auxDepthOptionsSub.callback_group = auxDepthCallbackGroup;
    rclcpp::SubscriptionOptions instanceMaskOptions;
    instanceMaskOptions.callback_group = instanceMaskCallbackGroup;
    rclcpp::SubscriptionOptions semanticOptions;
    semanticOptions.callback_group = semanticCallbackGroup;
    rclcpp::SubscriptionOptions voxbloxOptions;
    voxbloxOptions.callback_group = voxbloxCallbackGroup;

    auto subImu = node->create_subscription<Imu>(
        "/imu", sensorQos,
        [imugb](const Imu::ConstSharedPtr msg) { imugb->GrabImu(msg); },
        imuOptions);

    auto subImg = node->create_subscription<Image>(
        "/camera/image_raw", 1,
        [igb](const Image::ConstSharedPtr msg) { igb->GrabImage(msg); },
        imageOptions);

    rclcpp::Subscription<Image>::SharedPtr subAuxDepth;
    if (auxDepthOptions.use_aux_depth)
    {
        subAuxDepth = node->create_subscription<Image>(
            auxDepthTopic, 1,
            [igb](const Image::ConstSharedPtr msg) { igb->GrabAuxDepth(msg); },
            auxDepthOptionsSub);
        RCLCPP_INFO(node->get_logger(), "AuxDepth: subscribed to %s", auxDepthTopic.c_str());
    }

    auto subInstanceMask = node->create_subscription<Image>(
        segmentOptions.instance_mask_topic, 10,
        [igb](const Image::ConstSharedPtr msg) { igb->GrabInstanceMask(msg); },
        instanceMaskOptions);
    RCLCPP_INFO(node->get_logger(), "Instance masks: subscribed to %s", segmentOptions.instance_mask_topic.c_str());

    auto subSegmentedImage = node->create_subscription<segmenter_ros::msg::SegmenterDataMsg>(
        "/camera/color/image_segment", 50,
        [igb](const segmenter_ros::msg::SegmenterDataMsg::SharedPtr msg)
        { igb->GrabSegmentation(*msg); },
        semanticOptions);

    auto subVoxbloxSkeletonMesh = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/voxblox_skeletonizer/sparse_graph", 1,
        [igb](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        { igb->GrabVoxbloxSkeletonGraph(*msg); },
        voxbloxOptions);

    static std::shared_ptr<image_transport::ImageTransport> imageTransport =
        std::make_shared<image_transport::ImageTransport>(node);
    setupPublishers(node, imageTransport, node->get_name());
    setupServices(node, node->get_name());

    // --- Sync thread ---
    std::thread syncThread(&ImageGrabber::SyncWithImu, igb);

    // Dedicated executor + thread for imuCallbackGroup: isolates high-rate (~400Hz) IMU
    // servicing from the heavier per-frame vision callbacks (image/instanceMask/semantic/
    // voxblox), which share only 4 threads on the executor below and can otherwise starve the
    // IMU callback long enough to overrun sensorQos's buffer and silently drop samples.
    rclcpp::executors::SingleThreadedExecutor imuExecutor;
    imuExecutor.add_callback_group(imuCallbackGroup, node->get_node_base_interface());
    std::thread imuThread([&imuExecutor]() { imuExecutor.spin(); });

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    igb->mustStop = true;
    state->image_ready_cv.notify_all();  // unblock SyncWithImu if it's waiting
    syncThread.join();
    imuExecutor.cancel();
    imuThread.join();
    pSLAM->Shutdown();

    rclcpp::shutdown();
    return 0;
}
