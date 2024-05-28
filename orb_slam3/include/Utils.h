/**
 * 🚀 [vS-Graphs] A class for keeping the utility functions
 */

#ifndef UTILS_H
#define UTILS_H

#include "Tracking.h"
#include "Thirdparty/pcl_custom/WeightedSACSegmentation.hpp"

#include <cmath>
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

        // Variables
        static constexpr double DEG_TO_RAD = M_PI / 180.0;

        /**
         * @brief Calculate the Euclidean distance between two points
         * @param point1 first point
         * @param point2 second point
         */
        static double calculateEuclideanDistance(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);

        /**
         * @brief Calculate the distance between a point and a plane
         * @param plane the plane equation
         * @param point the given point
         */
        static double calculateDistancePointToPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &point);

        /**
         * @brief Checks to see if two planes are facing each other or not
         * @param plane1 first plane
         * @param plane2 second plane
         */
        static bool arePlanesFacingEachOther(const Plane *plane1, const Plane *plane2);

        /**
         * @brief Checks to see if two planes are perpendicular to each other or not
         * @param plane1 first plane
         * @param plane2 second plane
         * @param threshold the threshold value for perpendicularity
         */
        static bool arePlanesPerpendicular(const Plane *plane1, const Plane *plane2, const double &threshold);

        /**
         * @brief Returns the planes that are facing each other from the given list
         * @param planes list of planes to be checked
         */
        static std::vector<std::pair<Plane *, Plane *>> getAllPlanesFacingEachOther(const std::vector<Plane *> &planes);

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
         * @brief Gets the centeroid of a cluster of points
         * @param points the given cluster of points
         */
        static Eigen::Vector3d getClusterCenteroid(const std::vector<std::vector<Eigen::Vector3d *>> &points);

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
         * Downsamples the pointclouds based on the given leaf size
         * @param cloud the pointcloud to be downsampled
         */
        template <typename PointT>
        static typename pcl::PointCloud<PointT>::Ptr pointcloudDownsample(
            const typename pcl::PointCloud<PointT>::Ptr &cloud, float leafSize);

        /**
         * Filters the pointclouds based on the given min/max distance acceptable
         * @param cloud the pointcloud to be filtered
         * @param thresholds the acceptable min/max distance
         */
        template <typename PointT>
        static typename pcl::PointCloud<PointT>::Ptr pointcloudDistanceFilter(
            const typename pcl::PointCloud<PointT>::Ptr &cloud);

        /**
         * @brief Performs PCL ransac to get the plane equations from the a given point cloud
         * @param cloud the input point cloud
         * @param minSegmentationPoints the minimum number of points
         */
        template <typename PointT, template <typename> class SegmentationType>
        static std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>> ransacPlaneFitting(
            typename pcl::PointCloud<PointT>::Ptr &cloud);

        /**
         * @brief Checks to see if the given point is on the plane or not
         * @param planeEquation the plane equation
         * @param mapPoint the point to be checked
         */
        static bool pointOnPlane(Eigen::Vector4d planeEquation, MapPoint *mapPoint);

        /**
         * @brief associates given planes with the mapped planes
         * @param mappedPlanes the mapped planes
         * @param givenPlane the given plane
         * @return the plane id of the mapped plane
         */
        static int associatePlanes(const vector<Plane *> &mappedPlanes, g2o::Plane3D givenPlane);

        /**
         * @brief Gets the planeVariant type from the class id
         * @param clsId the class id
         * @return the planeVariant type
         */
        static ORB_SLAM3::Plane::planeVariant getPlaneTypeFromClassId(int clsId);

        /**
         * @brief Calculates the soft-min approximation of the given values
         * soft-min is an estimate of the quality of the segment based on per-pixel uncertainities
         * @param values the input values
         * @return the soft-min value
         */
        static double calcSoftMin(vector<double> &values);
    };
}

#endif // UTILS_H