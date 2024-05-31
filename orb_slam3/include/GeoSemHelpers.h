/**
 * 🚀 [vS-Graphs] A class for keeping common functions of Geometric and Semantic
 */

#ifndef GEOSEMHELPERS_H
#define GEOSEMHELPERS_H

#include "Atlas.h"
#include "Utils.h"

#include <Eigen/Core>

namespace ORB_SLAM3
{
    class GeoSemHelpers
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /**
         * @brief Creates a new plane object to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param pKF the address of the current keyframe
         * @param estimatedPlane the estimated plane
         * @param planeCloud the plane point cloud
         */
        static ORB_SLAM3::Plane *createMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                                const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud);

        /**
         * @brief Updates the map plane
         * @param mpAtlas the current map in Atlas
         * @param pKF the current keyframe
         * @param estimatedPlane the estimated plane
         * @param planeCloud the plane point cloud
         * @param planeId the plane id
         */
        static void updateMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId);

        /**
         * @brief Checks to see if the marker is attached to a door or not (e.g., a window)
         * and returns the name of it if exists (only valid for doors)
         * @param markerId the id of the marker
         * @param envDoors the list of doors in the environment
         */
        static std::pair<bool, std::string> checkIfMarkerIsDoor(const int &markerId,
                                                                std::vector<ORB_SLAM3::Door *> envDoors);

        /**
         * @brief Uses the detected markers to detect and map semantic objects, e.g., planes and doors
         * @param mpAtlas the current map in Atlas
         * @param pKF the current keyframe in which the detection took place
         * @param envDoors the list of doors in the environment
         * @param envRooms the list of rooms in the environment
         */
        static void markerSemanticAnalysis(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                           std::vector<ORB_SLAM3::Door *> envDoors,
                                           std::vector<ORB_SLAM3::Room *> envRooms);

        /**
         * @brief Creates a new marker object to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param pKF the address of the current keyframe
         * @param visitedMarker the address of the visited marker
         */
        static Marker *createMapMarker(Atlas *mpAtlas, KeyFrame *pKF, const Marker *visitedMarker);

        /**
         * @brief Creates a new door object to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param pKF the address of the current keyframe
         * @param attachedMarker the address of the attached marker
         * @param name the name of the door
         */
        static void createMapDoor(Atlas *mpAtlas, KeyFrame *pKF, Marker *attachedMarker, std::string name);

        /**
         * @brief Organizes the walls of a four-walled room
         * @param givenRoom the address of the detected room
         */
        static void organizeRoomWalls(Room *givenRoom);

        /**
         * @brief Creates a new room object (corridor or room) to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param matchedRoom the address of the room matched from the database
         * @param attachedMarker the address of the attached marker
         */
        static void createMapRoomCandidateByMarker(Atlas *mpAtlas, Room *matchedRoom, Marker *attachedMarker);

        /**
         * @brief Creates a new room object (corridor or room) to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param isCorridor the boolean value to check if the room is a corridor or not
         * @param clusterCentroid the centroid of the cluster
         * @param walls the vector of walls detected in the room
         */
        static void createMapRoomCandidateByFreeSpace(Atlas *mpAtlas, bool isCorridor,
                                                      std::vector<ORB_SLAM3::Plane *> walls,
                                                      Eigen::Vector3d clusterCentroid = Eigen::Vector3d::Zero());
    };
}

#endif // GEOSEMHELPERS_H