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
        double static calculateEuclideanDistance(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);
    };
}

#endif // UTILS_H