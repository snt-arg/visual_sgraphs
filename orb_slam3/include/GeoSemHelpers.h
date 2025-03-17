/**
 * 🚀 [vS-Graphs] A class for keeping common functions of Geometric and Semantic
 */

#ifndef GEOSEMHELPERS_H
#define GEOSEMHELPERS_H

#include "Atlas.h"
#include "Utils.h"

#include <Eigen/Core>

namespace VS_GRAPHS
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
         * @param semanticType the semantic type of the plane observation
         * @param confidence the confidence of the plane observation
         */
        static VS_GRAPHS::Plane *createMapPlane(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                                const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                                                VS_GRAPHS::Plane::planeVariant semanticType = VS_GRAPHS::Plane::planeVariant::UNDEFINED,
                                                double confidence = 1.0);

        /**
         * @brief Updates the map plane
         * @param mpAtlas the current map in Atlas
         * @param pKF the current keyframe
         * @param estimatedPlane the estimated plane
         * @param planeCloud the plane point cloud
         * @param planeId the plane id
         * @param semanticType the semantic type of the plane observation
         * @param confidence the confidence of the plane observation
         */
        static void updateMapPlane(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId,
                                   VS_GRAPHS::Plane::planeVariant semanticType = VS_GRAPHS::Plane::planeVariant::UNDEFINED,
                                   double confidence = 1.0);

        /**
         * @brief Checks to see if the marker is attached to a door or not (e.g., a window)
         * and returns the name of it if exists (only valid for doors)
         * @param markerId the id of the marker
         * @param envDoors the list of doors in the environment
         */
        static std::pair<bool, std::string> checkIfMarkerIsDoor(const int &markerId,
                                                                std::vector<VS_GRAPHS::Door *> envDoors);

        /**
         * @brief Uses the detected markers to detect and map semantic objects, e.g., planes and doors
         * @param mpAtlas the current map in Atlas
         * @param pKF the current keyframe in which the detection took place
         * @param envDoors the list of doors in the environment
         * @param envRooms the list of rooms in the environment
         */
        static void markerSemanticAnalysis(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF,
                                           std::vector<VS_GRAPHS::Door *> envDoors,
                                           std::vector<VS_GRAPHS::Room *> envRooms);

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
         * @param walls the vector of walls detected in the room
         * @param clusterCentroid the centroid of the cluster
         */
        static VS_GRAPHS::Room *createMapRoomCandidateByFreeSpace(Atlas *mpAtlas, bool isCorridor,
                                                                  std::vector<VS_GRAPHS::Plane *> walls,
                                                                  Eigen::Vector3d clusterCentroid = Eigen::Vector3d::Zero());

        /**
         * @brief Updates the room object in the map with new room information
         * @param markerBasedRoom the address of the detected marker-based room
         * @param clusterBasedRoom the address of the detected cluster-based room
         * @param isMarkerBasedMapped the boolean value to check if we augment from marker-based or the cluster-based room
         */
        static void augmentMapRoomCandidate(VS_GRAPHS::Room *markerBasedRoom, VS_GRAPHS::Room *clusterBasedRoom,
                                            bool isMarkerBasedMapped);

        /**
         * @brief Chooses a ground plane from the Atlas to be associated with the room
         * @param mpAtlas the current map in Atlas
         * @param givenRoom the address of the detected room
         */
        static void associateGroundPlaneToRoom(Atlas *mpAtlas, VS_GRAPHS::Room *givenRoom);

        /**
         * @brief Counts the number of points in the ground plane that are within the walls of the room
         * @param roomWalls the vector of walls detected in the room
         * @param groundPlane the ground plane associated with the room
         */
        static size_t countGroundPlanePointsWithinWalls(std::vector<VS_GRAPHS::Plane *> &roomWalls, VS_GRAPHS::Plane *groundPlane);
    };
}

#endif // GEOSEMHELPERS_H