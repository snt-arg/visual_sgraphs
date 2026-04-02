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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Creates a new plane object to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param pKF the address of the current keyframe
         * @param estimatedPlane the estimated plane
         * @param planeCloud the plane point cloud
         * @param semanticType the semantic type of the plane observation
         * @param confidence the confidence of the plane observation
         */
        static ORB_SLAM3::Plane *createMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                                const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                                                ORB_SLAM3::Plane::planeVariant semanticType = ORB_SLAM3::Plane::planeVariant::UNDEFINED,
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
        static void updateMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId,
                                   ORB_SLAM3::Plane::planeVariant semanticType = ORB_SLAM3::Plane::planeVariant::UNDEFINED,
                                   double confidence = 1.0);

        /**
         * @brief Checks to see if the marker is attached to a doorway or not (e.g., a window)
         * and returns the name of it if exists (only valid for doors)
         * @param markerId the id of the marker
         * @param envDoorways the list of doorways in the environment
         */
        static std::pair<bool, std::string> checkIfMarkerIsDoorway(const int &markerId,
                                                                std::vector<ORB_SLAM3::Room *> envRooms);

        /**
         * @brief Uses the detected markers to detect and map semantic objects, e.g., planes and doors
         * @param mpAtlas the current map in Atlas
         * @param pKF the current keyframe in which the detection took place
         * @param envRooms the list of rooms in the environment
         */
        static void markerSemanticAnalysis(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                           std::vector<ORB_SLAM3::Room *> envRooms);

        /**
         * @brief Creates a new marker object to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param pKF the address of the current keyframe
         * @param visitedMarker the address of the visited marker
         */
        static Marker *createMapMarker(Atlas *mpAtlas, KeyFrame *pKF, const Marker *visitedMarker);

        /**
         * @brief Creates a new doorway object to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param pKF the address of the current keyframe
         */
        static void createMapDoorway(ORB_SLAM3::Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF);

        /**
         * @brief Organizes the walls of a four-walled room
         * @param givenRoom the address of the detected room
         */
        static void organizeRoomWalls(Room *givenRoom);

        /**
         * @brief Creates a blank room object (undefined variant) to be added to the map
         * @param mpAtlas the current map in Atlas
         * @param centroid the centroid of the room (optional)
         */
        static ORB_SLAM3::Room *createBlankRoomCandidate(Atlas *mpAtlas,
                                                         Eigen::Vector3d centroid = Eigen::Vector3d::Zero());

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
        static ORB_SLAM3::Room *createMapRoomCandidateByFreeSpace(Atlas *mpAtlas, bool isCorridor,
                                                                  std::vector<ORB_SLAM3::Plane *> walls,
                                                                  Eigen::Vector3d clusterCentroid = Eigen::Vector3d::Zero());

        /**
         * @brief Updates the room object in the map with new room information
         * @param markerBasedRoom the address of the detected marker-based room
         * @param clusterBasedRoom the address of the detected cluster-based room
         * @param isMarkerBasedMapped the boolean value to check if we augment from marker-based or the cluster-based room
         */
        static void augmentMapRoomCandidate(ORB_SLAM3::Room *markerBasedRoom, ORB_SLAM3::Room *clusterBasedRoom,
                                            bool isMarkerBasedMapped);

        /**
         * @brief Chooses a ground plane from the Atlas to be associated with the room
         * @param mpAtlas the current map in Atlas
         * @param givenRoom the address of the detected room
         */
        static void associateGroundPlaneToRoom(Atlas *mpAtlas, ORB_SLAM3::Room *givenRoom);

        /**
         * @brief Counts the number of points in the ground plane that are within the walls of the room
         * @param roomWalls the vector of walls detected in the room
         * @param groundPlane the ground plane associated with the room
         */
        static size_t countGroundPlanePointsWithinWalls(std::vector<ORB_SLAM3::Plane *> &roomWalls, ORB_SLAM3::Plane *groundPlane);

        /**
         * @brief Creates a new floor object to be added to the map
         * @param mpAtlas the current map in Atlas
         */
        static void createMapFloor(ORB_SLAM3::Atlas *mpAtlas);
    };
}

#endif // GEOSEMHELPERS_H