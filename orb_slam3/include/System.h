/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"
#include "Semantic/Marker.h"
#include "Semantic/Door.h"
#include "Semantic/Room.h"
#include "Geometric/Plane.h"
#include "Types/SystemParams.h"
#include "SemanticSegmentation.h"
#include "GeometricSegmentation.h"

namespace ORB_SLAM3
{

    class Verbose
    {
    public:
        enum eLevel
        {
            VERBOSITY_QUIET = 0,
            VERBOSITY_NORMAL = 1,
            VERBOSITY_VERBOSE = 2,
            VERBOSITY_VERY_VERBOSE = 3,
            VERBOSITY_DEBUG = 4
        };

        static eLevel th;

    public:
        static void PrintMess(std::string str, eLevel lev)
        {
            if (lev <= th)
                cout << str << endl;
        }

        static void SetTh(eLevel _th)
        {
            th = _th;
        }
    };

    class Viewer;
    class FrameDrawer;
    class MapDrawer;
    class Atlas;
    class Tracking;
    class LocalMapping;
    class LoopClosing;
    class Settings;
    class SemanticSegmentation;
    class GeometricSegmentation;

    class System
    {
    public:
        // Input sensor
        enum eSensor
        {
            NOT_SET = -1,
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2,
            IMU_MONOCULAR = 3,
            IMU_STEREO = 4,
            IMU_RGBD = 5,
        };

        // File type
        enum FileType
        {
            TEXT_FILE = 0,
            BINARY_FILE = 1,
        };

        // ROS parameters set in 'common.h' using launch files
        SystemParams sysParams;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System(const string &strVocFile, const string &strSettingsFile, const SystemParams &params, const eSensor sensor,
               const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

        /**
         * @brief Process the given stereo frame for tracking. Images must be synchronized and rectified.
         * @param imLeft the input RGB image (CV_8UC3) or grayscale (CV_8U) from the left camera
         * @param imRight the input RGB image (CV_8UC3) or grayscale (CV_8U) from the right camera
         * @param timestamp the timestamp of the frame
         * @param vImuMeas the vector of IMU measurements
         * @param filename the name of the file
         * @param markers the vector of fiducial markers
         * @param envDoors the vector of doors
         * @param envRooms the vector of rooms
         * @return the camera pose (empty if tracking fails)
         */
        Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp,
                                 const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(), string filename = "",
                                 const vector<Marker *> markers = vector<Marker *>{}, const vector<Door *> envDoors = vector<Door *>{},
                                 const vector<Room *> envRooms = vector<Room *>{});

        /**
         * @brief Process the given rgbd frame for tracking. The DepthMap must be registered to the RGB frame.
         * @param im the input RGB image (CV_8UC3) or grayscale (CV_8U)
         * @param depthmap the input DepthMap (CV_32F)
         * @param mainCloud the main input PointCloud before filtering
         * @param timestamp the timestamp of the frame
         * @param vImuMeas the vector of IMU measurements
         * @param filename the name of the file
         * @param markers the vector of fiducial markers
         * @param envDoors the vector of doors
         * @param envRooms the vector of rooms
         * @return the camera pose (empty if tracking fails)
         */
        Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap,
                               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mainCloud,
                               const double &timestamp, const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                               string filename = "", const vector<Marker *> markers = vector<Marker *>{},
                               const vector<Door *> envDoors = vector<Door *>{},
                               const vector<Room *> envRooms = vector<Room *>{});

        /**
         * @brief Process the given stereo frame for tracking. Images must be synchronized and rectified.
         * @param im the input RGB image (CV_8UC3) or grayscale (CV_8U) from the left camera
         * @param timestamp the timestamp of the frame
         * @param vImuMeas the vector of IMU measurements
         * @param filename the name of the file
         * @param markers the vector of fiducial markers
         * @param envDoors the vector of doors
         * @param envRooms the vector of rooms
         * @return the camera pose (empty if tracking fails)
         */
        Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp,
                                    const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(), string filename = "",
                                    const vector<Marker *> markers = vector<Marker *>{}, const vector<Door *> envDoors = vector<Door *>{},
                                    const vector<Room *> envRooms = vector<Room *>{});

        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();

        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear Atlas or the active map)
        void Reset();
        void ResetActiveMap();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();
        bool isShutDown();

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        void SaveTrajectoryEuRoC(const string &filename);
        void SaveKeyFrameTrajectoryEuRoC(const string &filename);

        void SaveTrajectoryEuRoC(const string &filename, Map *pMap);
        void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap);

        // Save data used for initialization debug
        void SaveDebugData(const int &iniIdx);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);

        // TODO: Save/Load functions
        bool SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();
        cv::Mat GetCurrentFrame();
        std::vector<Door *> GetAllDoors();
        std::vector<Room *> GetAllRooms();
        std::vector<Plane *> GetAllPlanes();
        std::vector<Marker *> GetAllMarkers();
        std::vector<KeyFrame *> GetAllKeyFrames();
        std::vector<MapPoint *> GetAllMapPoints();
        std::vector<MapPoint *> GetTrackedMapPoints();
        std::vector<Sophus::SE3f> GetAllKeyframePoses();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        Sophus::SE3f GetCamTwc();
        Sophus::SE3f GetImuTwb();
        Eigen::Vector3f GetImuVwb();
        bool isImuPreintegrated();

        // For accessing ROS parameters, set in common.h
        SystemParams GetSystemParameters();

        // For debugging
        double GetTimeFromIMUInit();
        bool isLost();
        bool isFinished();

        void ChangeDataset();

        float GetImageScale();

        /**
         * @brief Set the vector of doors and rooms fetched from the database for GeometricSegmentation
         * @param envDoors the vector of doors fetched from the database
         * @param envRooms the vector of rooms fetched from the database
         */
        void setEnvFetchedValues(std::vector<Door *> envDoors, std::vector<Room *> envRooms);

        /**
         * @brief Add the segmented image to the buffer in the SemanticSegmentation
         * @param tuple the address of the tuple of segmented image and pointcloud
         */
        void addSegmentedImage(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple);

#ifdef REGISTER_TIMES
        void InsertRectTime(double &time);
        void InsertResizeTime(double &time);
        void InsertTrackTime(double &time);
#endif

    private:
        bool SaveAtlas(int type);
        bool LoadAtlas(int type);

        string CalculateCheckSum(string filename, int type);

        // Input sensor
        eSensor mSensor;

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        // Map* mpMap;
        Atlas *mpAtlas;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking *mpTracker;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping *mpLocalMapper;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        LoopClosing *mpLoopCloser;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer *mpViewer;

        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;

        // Geometric & Semantic Segmentation
        SemanticSegmentation *mpSemanticSegmentation;
        GeometricSegmentation *mpGeometricSegmentation;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // 🚀 [vS-Graphs v.2.0] Two new threads: Geometric Segmentation and Semantic Segmentation
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread *mptViewer;
        std::thread *mptLoopClosing;
        std::thread *mptLocalMapping;
        std::thread *mptSemanticSegmentation;
        std::thread *mptGeometricSegmentation;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;
        bool mbResetActiveMap;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Shutdown flag
        bool mbShutDown;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::mutex mMutexState;

        //
        string mStrLoadAtlasFromFile;
        string mStrSaveAtlasToFile;

        string mStrVocabularyFilePath;

        Settings *settings_;
    };

} // namespace ORB_SLAM

#endif // SYSTEM_H
