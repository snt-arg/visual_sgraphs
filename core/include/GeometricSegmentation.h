/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details: https://www.gnu.org/licenses/
*/

#ifndef GEOMETRICSEG_H
#define GEOMETRICSEG_H

#include "Atlas.h"
#include "Utils.h"
#include "GeoSemHelpers.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

namespace ORB_SLAM3
{
    class Atlas;

    class GeometricSegmentation
    {
    private:
        Atlas *mpAtlas;
        bool mHasDepthCloud;
        std::mutex mMutexNewKFs;
        std::list<KeyFrame *> mvpKeyFrameBuffer;
        std::vector<ORB_SLAM3::Room *> envRooms;

        // system parameters
        SystemParams *sysParams;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GeometricSegmentation(Atlas *pAtlas, bool hasDepthCloud,
                              std::vector<ORB_SLAM3::Room *> envRooms);

        void AddKeyFrameToBuffer(KeyFrame *pKF);
        std::list<KeyFrame *> GetKeyFrameBuffer();

        /**
         * @brief Detects all the planes in the current keyframe
         * @param pKF the address of the current keyframe
         * @param hasDepthCloud a boolean to indicate if the point cloud has depth information
         * @param minCloudSize the minimum size of the point cloud to be segmented
         */
        void fetchPlanesFromKeyFrame(ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud);

        /**
         * @brief Calculation of plane equation from point clouds (provided by depth in RGB-D or calculated from
         * map points in Monocular and Stereo)
         * @param pKF the current KeyFrame
         * @param hasDepthCloud a boolean to indicate if the point cloud has depth information
         * @param minCloudSize the minimum size of the point cloud to be segmented
         */
        std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
        getPlanesFromPointClouds(ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud);

        /**
         * @brief Get the point cloud from a set of map-points
         * @param points the set of map-points
         */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudFromSparsePoints(const std::vector<MapPoint *> &points);

        /**
         * @brief Calculation of the equation of the plane from marker pose
         * @param rotationMatrix the rotation matrix
         * @param translation the translation matrix
         */
        Eigen::Vector4d getPlaneEquationFromPose(const Eigen::Matrix3f &rotationMatrix,
                                                 const Eigen::Vector3f &translation);

        /**
         * @brief Runs the geometric segmentation thread
         */
        void Run();
    };
}

#endif // GEOMETRICSEG_H