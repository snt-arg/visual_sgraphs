/**
 * 🚀 [vS-Graphs] A class for keeping the utility functions
 */

#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>

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
    };
}

#endif // UTILS_H