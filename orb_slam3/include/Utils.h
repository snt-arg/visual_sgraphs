/**
 * 🚀 [vS-Graphs] A class for keeping the utility functions
 */

#ifndef UTILS_H
#define UTILS_H

#include "Tracking.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace ORB_SLAM3
{
    class Utils
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /**
         * @brief Calculate the Euclidean distance between two points
         * @param point1 first point
         * @param point2 second point
         */
        static double calculateEuclideanDistance(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);

        /**
         * @brief Corrects the given plane equations to apply calculations
         * @param plane the input plane
         */
        static Eigen::Vector4d correctPlaneDirection(const Eigen::Vector4d &plane);

        /**
         * @brief Converts the plane equation from local to global
         * @param kfPose the pose of the current keyframe
         * @param plane the plane equation in the local map
         */
        static g2o::Plane3D convertToGlobalEquation(const Eigen::Matrix4d &kfPose, const g2o::Plane3D &plane);

        /**
         * @brief Gets the center points of a room with two walls
         * @param markerPosition the position of the marker
         * @param wall1 the first plane
         * @param wall2 the second plane
         */
        static Eigen::Vector3d getRoomCenter(const Eigen::Vector3d &markerPosition,
                                             const Eigen::Vector4d &wall1,
                                             const Eigen::Vector4d &wall2);

        /**
         * @brief Gets the center points of a room with four walls
         * @param x_plane1 the first plane in X direction
         * @param x_plane2 the second plane in X direction
         * @param y_plane1 the first plane in Y direction
         * @param y_plane2 the second plane in Y direction
         */
        static Eigen::Vector3d getRoomCenter(const Eigen::Vector4d x_plane1, const Eigen::Vector4d x_plane2,
                                             const Eigen::Vector4d y_plane1, const Eigen::Vector4d y_plane2);

        /**
         * @brief Checks to see if the marker is attached to a door or not (e.g., a window)
         * and returns the name of it if exists (only valid for doors)
         * @param markerId the id of the marker
         */
        static std::pair<bool, std::string> isMarkerAttachedToDoor(const int &markerId,
                                                                   std::vector<ORB_SLAM3::Door *> envDoors);

        /**
         * Filters the pointclouds based on the given min/max distance between the points
         * @param cloud the pointcloud to be filtered
         */
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudDistanceFilter(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

        /**
         * Downsamples the pointclouds based on the given leaf size
         * @param cloud the pointcloud to be downsampled
         */
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudDownsample(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

        /**
         * @brief Performs PCL ransac to get the plane equations from the a given point cloud
         * @param cloud the input point cloud
         * @param minSegmentationPoints the minimum number of points
         */
        static std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> ransacPlaneFitting(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int minSegmentationPoints);
    };
}

#endif // UTILS_H