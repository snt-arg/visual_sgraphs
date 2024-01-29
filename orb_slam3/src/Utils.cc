#include "Utils.h"

namespace ORB_SLAM3
{
    double Utils::calculateEuclideanDistance(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2)
    {
        double dx = p1.x() - p2.x();
        double dy = p1.y() - p2.y();
        double dz = p1.z() - p2.z();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Eigen::Vector4d Utils::correctPlaneDirection(const Eigen::Vector4d &plane)
    {
        // Check if the transformation is needed
        if (plane(3) > 0)
            return -plane;
        else
            return plane;
    }

    Eigen::Vector3d Utils::getRoomCenter(const Eigen::Vector3d &markerPosition,
                                         const Eigen::Vector4d &wall1,
                                         const Eigen::Vector4d &wall2)
    {
        Eigen::Vector3d roomCenter;
        Eigen::Vector3d vec, vectorNormal;

        // Get the dominant wall by comparing the magnitudes of the last elements of the given walls
        if (fabs(wall1(3)) > fabs(wall2(3)))
            // Calculate the midpoint of the dominant wall
            vec = (0.5 * (fabs(wall1(3)) * wall1.head(3) - fabs(wall2(3)) * wall2.head(3))) +
                  fabs(wall2(3)) * wall2.head(3);
        else
            // Calculate the midpoint of the dominant wall
            vec = (0.5 * (fabs(wall2(3)) * wall2.head(3) - fabs(wall1(3)) * wall1.head(3))) +
                  fabs(wall1(3)) * wall1.head(3);

        // Normalize the vector to obtain the normal direction of the room
        vectorNormal = vec / vec.norm();

        // Calculate the room center by projecting the marker position onto the room plane
        roomCenter = vec + (markerPosition - (markerPosition.dot(vectorNormal)) * vectorNormal);

        return roomCenter;
    }

    Eigen::Vector3d Utils::getRoomCenter(const Eigen::Vector4d x_plane1, const Eigen::Vector4d x_plane2,
                                         const Eigen::Vector4d y_plane1, const Eigen::Vector4d y_plane2)
    {
        Eigen::Vector3d roomCenter;
        Eigen::Vector3d vectorX, vectorY;

        // Calculate the midpoint vector along the x-axis of the room
        if (fabs(x_plane1(3)) > fabs(x_plane2(3)))
            vectorX = (0.5 * (fabs(x_plane1(3)) * x_plane1.head(3) - fabs(x_plane2(3)) * x_plane2.head(3))) +
                      fabs(x_plane2(3)) * x_plane2.head(3);
        else
            vectorX = (0.5 * (fabs(x_plane2(3)) * x_plane2.head(3) - fabs(x_plane1(3)) * x_plane1.head(3))) +
                      fabs(x_plane1(3)) * x_plane1.head(3);

        // Calculate the midpoint vector along the y-axis of the room
        if (fabs(y_plane1(3)) > fabs(y_plane2(3)))
            vectorY = (0.5 * (fabs(y_plane1(3)) * y_plane1.head(3) - fabs(y_plane2(3)) * y_plane2.head(3))) +
                      fabs(y_plane2(3)) * y_plane2.head(3);
        else
            vectorY = (0.5 * (fabs(y_plane2(3)) * y_plane2.head(3) - fabs(y_plane1(3)) * y_plane1.head(3))) +
                      fabs(y_plane1(3)) * y_plane1.head(3);

        // Calculate the room center by summing the midpoint vectors along the x and y axes
        roomCenter = vectorX + vectorY;

        return roomCenter;
    }
}