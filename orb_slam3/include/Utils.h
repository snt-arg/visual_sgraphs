/**
 * 🚀 [vS-Graphs] A class for keeping the utility functions
 */

#ifndef UTILS_H
#define UTILS_H

#include "Atlas.h"
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace VS_GRAPHS
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
         * @brief Calculates the intersection point of a line and a plane
         * @param plane the plane equation
         * @param lineStart the start point of the line
         * @param lineEnd the end point of the line
         */
        static Eigen::Vector3d lineIntersectsPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &lineStart,
                                                   const Eigen::Vector3d &lineEnd);

        /**
         * @brief Checks to see if two planes are apart enough from each other, given a threshold
         * @param plane1 first plane
         * @param plane2 second plane
         * @param threshold the threshold value for perpendicularity
         */
        static bool arePlanesApartEnough(const Plane *plane1, const Plane *plane2, const double &threshold);

        /**
         * @brief Checks to see if two planes are perpendicular to each other or not
         * @param plane1 first plane
         * @param plane2 second plane
         * @param threshold the threshold value for perpendicularity
         */
        static bool arePlanesPerpendicular(const Plane *plane1, const Plane *plane2, const double &threshold);

        /**
         * @brief Checks to see if two planes are facing each other or not
         * @param plane1 first plane
         * @param plane2 second plane
         */
        static bool arePlanesFacingEachOther(const Plane *plane1, const Plane *plane2);

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
        static g2o::Plane3D applyPoseToPlane(const Eigen::Matrix4d &kfPose, const g2o::Plane3D &plane);

        /**
         * @brief Gets the centeroid of a cluster of points
         * @param points the given cluster of points
         */
        static Eigen::Vector3d getClusterCenteroid(const std::vector<Eigen::Vector3d> &points);

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
         * @brief Downsamples the pointclouds based on the given leaf size
         *
         * @param cloud the pointcloud to be downsampled
         * @param leafSize the leaf size for downsampling
         */
        template <typename PointT>
        static typename pcl::PointCloud<PointT>::Ptr pointcloudDownsample(
            const typename pcl::PointCloud<PointT>::Ptr &cloud, const float leafSize, const unsigned int minPointsPerVoxel);

        /**
         * @brief Filters the pointclouds based on the given min/max distance acceptable
         *
         * @param cloud the pointcloud to be filtered
         */
        template <typename PointT>
        static typename pcl::PointCloud<PointT>::Ptr pointcloudDistanceFilter(
            const typename pcl::PointCloud<PointT>::Ptr &cloud);

        /**
         * @brief Removes the points that are farther away from their neighbors
         *
         * @param cloud the pointcloud to be filtered
         * @param meanThresh the mean threshold for neighbor points
         * @param stdDevThresh the standard deviation threshold for neighbor points
         */
        template <typename PointT>
        static typename pcl::PointCloud<PointT>::Ptr pointcloudOutlierRemoval(
            const typename pcl::PointCloud<PointT>::Ptr &cloud, const int meanThresh, const float stdDevThresh);

        /**
         * @brief Performs PCL ransac to get the plane equations from the a given point cloud
         *
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
         * @param givenPlane the given plane (in the same frame as the kfPose, global if kfPose is identity)
         * @param kfPose the pose of the current keyframe
         * @param threshold the threshold value for association
         * @return the plane id of the mapped plane
         */
        static int associatePlanes(const vector<Plane *> &mappedPlanes, g2o::Plane3D givenPlane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr givenCloud,
                                   const Eigen::Matrix4d &kfPose, const Plane::planeVariant obsPlaneType, const float threshold);

        /**
         * @brief Clusters the point cloud into separate clouds based on the plane detection
         * @param cloud the point cloud to be clustered
         * @param clusterIndices the vector of point indices for each cluster
         */
        static void clusterPlaneClouds(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, std::vector<pcl::PointIndices> &clusterIndices);

        /**
         * @brief Re-associates semantically classified planes if they get closer after optimization
         * @param mpAtlas a pointer to the Atlas
         */
        static void reAssociateSemanticPlanes(Atlas *mpAtlas);

        /**
         * @brief Gets the planeVariant type from the class id
         * @param clsId the class id
         * @return the planeVariant type
         */
        static VS_GRAPHS::Plane::planeVariant getPlaneTypeFromClassId(int clsId);

        /**
         * @brief Gets the class id from the planeVariant type
         * @param planeType the planeVariant type
         * @return the class id
         */
        static int getClassIdFromPlaneType(VS_GRAPHS::Plane::planeVariant planeType);

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