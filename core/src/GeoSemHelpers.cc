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

#include "GeoSemHelpers.h"

namespace ORB_SLAM3
{
    ORB_SLAM3::Plane *GeoSemHelpers::createMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                                    const g2o::Plane3D estimatedPlane,
                                                    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                                                    ORB_SLAM3::Plane::planeVariant semanticType, double confidence)
    {
        ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());
        newMapPlane->refKeyFrame = pKF;

        // The observation of the plane
        ORB_SLAM3::Plane::Observation obs;

        // the observation of the plane equation
        obs.localPlane = estimatedPlane;

        // the observation of the plane point cloud (measurement)
        Eigen::Matrix4d Gij;
        Gij.setZero();
        if (SystemParams::GetParams()->optimization.plane_point.enabled)
        {
            for (auto &point : planeCloud->points)
            {
                Eigen::Vector4d pointVec;
                pointVec << point.x, point.y, point.z, 1;
                Gij += pointVec * pointVec.transpose();
            }
        }
        obs.Gij = Gij;

        // the semantic class of the observation
        obs.semanticType = semanticType;

        // the aggregated confidence of the plane
        obs.confidence = confidence;
        newMapPlane->addObservation(pKF, obs);

        // Set the plane type to undefined, as it is not known yet
        newMapPlane->setPlaneType(ORB_SLAM3::Plane::planeVariant::UNDEFINED);

        // Set the global equation of the plane
        g2o::Plane3D globalEquation = Utils::applyPoseToPlane(pKF->GetPoseInverse().matrix().cast<double>(),
                                                              estimatedPlane);
        newMapPlane->setGlobalEquation(globalEquation);

        // transform the plane cloud to the global frame
        pcl::transformPointCloud(*planeCloud, *planeCloud, pKF->GetPoseInverse().matrix().cast<float>());

        // Fill the plane with the pointcloud
        if (!planeCloud->points.empty())
            newMapPlane->setMapClouds(planeCloud);

        if (SystemParams::GetParams()->optimization.plane_map_point.enabled)
        {
            for (const auto &mapPoint : pKF->GetMapPoints())
                if (newMapPlane->isPointinPlaneCloud(mapPoint->GetWorldPos().cast<double>()))
                    newMapPlane->setMapPoints(mapPoint);
        }

        pKF->AddMapPlane(newMapPlane);
        mpAtlas->AddMapPlane(newMapPlane);

        return newMapPlane;
    }

    void GeoSemHelpers::updateMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId,
                                       ORB_SLAM3::Plane::planeVariant semanticType, double confidence)
    {
        // Find the matched plane among all planes of the map
        Plane *currentPlane = mpAtlas->GetPlaneById(planeId);

        // The observation of the plane
        ORB_SLAM3::Plane::Observation obs;

        // the observation of the plane equation
        obs.localPlane = estimatedPlane;

        // the observation of the plane point cloud (measurement)
        Eigen::Matrix4d Gij;
        Gij.setZero();
        if (SystemParams::GetParams()->optimization.plane_point.enabled)
        {
            for (auto &point : planeCloud->points)
            {
                Eigen::Vector4d pointVec;
                pointVec << point.x, point.y, point.z, 1;
                Gij += pointVec * pointVec.transpose() * (static_cast<int>(point.a) / 255.0);
            }
        }
        obs.Gij = Gij;

        // the semantic class of the observation
        obs.semanticType = semanticType;

        // the aggregated confidence of the plane
        obs.confidence = confidence;
        currentPlane->addObservation(pKF, obs);

        // Add the plane to the list of planes in the current KeyFrame
        pKF->AddMapPlane(currentPlane);

        // transform the plane cloud to the global frame
        pcl::transformPointCloud(*planeCloud, *planeCloud, pKF->GetPoseInverse().matrix().cast<float>());

        // Update the pointcloud of the plane
        if (!planeCloud->points.empty())
            currentPlane->setMapClouds(planeCloud);

        if (SystemParams::GetParams()->optimization.plane_map_point.enabled)
        {
            for (const auto &mapPoint : pKF->GetMapPoints())
                if (currentPlane->isPointinPlaneCloud(mapPoint->GetWorldPos().cast<double>()))
                    currentPlane->setMapPoints(mapPoint);
        }
    }

    std::pair<bool, std::string> GeoSemHelpers::checkIfMarkerIsDoorway(const int &markerId,
                                                                       std::vector<ORB_SLAM3::Room *> envRooms)
    {
        bool isDoorway = true;
        std::string name = "";
        // Loop over all markers attached to doorways
        for (const auto &roomPtr : envRooms)
        {
            if (roomPtr->getMetaMarkerId() == markerId)
            {
                isDoorway = false;
                name = roomPtr->getName();
                break; // No need to continue searching if found
            }
        }
        // Returning
        return std::make_pair(isDoorway, name);
    }

    void GeoSemHelpers::markerSemanticAnalysis(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                               std::vector<ORB_SLAM3::Room *> envRooms)
    {
        // Get the markers from the current KeyFrame
        std::vector<Marker *> mvpMapMarkers = pKF->getCurrentFrameMarkers();

        for (Marker *mCurrentMarker : mvpMapMarkers)
        {
            // Variables
            ORB_SLAM3::Marker *currentMapMarker;

            // Check the type of the marker
            std::pair<bool, std::string> result = checkIfMarkerIsDoorway(mCurrentMarker->getId(), envRooms);
            bool markerIsDoorway = result.first;
            std::string doorwayName = result.second;

            // Change the marker type
            mCurrentMarker->setMarkerType(markerIsDoorway
                                              ? ORB_SLAM3::Marker::markerVariant::ON_DOOR
                                              : ORB_SLAM3::Marker::markerVariant::ON_ROOM_CENTER);

            // If the marker is not in the map, add it
            if (!mCurrentMarker->isMarkerInGMap())
            {
                mCurrentMarker->setMap(mpAtlas->GetCurrentMap());
                mCurrentMarker->setGlobalPose(pKF->GetPoseInverse() * mCurrentMarker->getLocalPose());
                mCurrentMarker->setMarkerInGMap(true);

                // Creating a new marker in the map
                currentMapMarker = createMapMarker(mpAtlas, pKF, mCurrentMarker);
            }
            // Else, add the observation to the existing marker
            else
                for (auto mappedMarker : mpAtlas->GetAllMarkers())
                    if (mappedMarker->getId() == mCurrentMarker->getId())
                    {
                        currentMapMarker = mappedMarker;
                        currentMapMarker->addObservation(pKF, mCurrentMarker->getLocalPose());
                    }

            // Decide based on the marker type
            if (!markerIsDoorway)
            {
                // The current marker is a room meta-marker
                ORB_SLAM3::Room *mappedRoom;

                // Check to find the real room values fetched from the JSON file
                for (Room *envRoom : envRooms)
                    // Check if the current detected marker belongs to this room
                    if (mCurrentMarker->getId() == envRoom->getMetaMarkerId())
                        // Create a room candidate if does not exist in the map
                        // [💡hint] Creation of room candidates happens here and updating (changing to real candidates)
                        // happens in the Semantic Segmentation module
                        createMapRoomCandidateByMarker(mpAtlas, envRoom, mCurrentMarker);
            }
        }
    }

    ORB_SLAM3::Marker *GeoSemHelpers::createMapMarker(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                                      const ORB_SLAM3::Marker *visitedMarker)
    {
        ORB_SLAM3::Marker *newMapMarker = new ORB_SLAM3::Marker();

        newMapMarker->setId(visitedMarker->getId());
        newMapMarker->setMap(mpAtlas->GetCurrentMap());
        newMapMarker->setOpId(visitedMarker->getOpId());
        newMapMarker->setTime(visitedMarker->getTime());
        newMapMarker->setLocalPose(visitedMarker->getLocalPose());
        newMapMarker->setGlobalPose(visitedMarker->getGlobalPose());
        newMapMarker->setMarkerType(visitedMarker->getMarkerType());
        newMapMarker->setMarkerInGMap(visitedMarker->isMarkerInGMap());
        newMapMarker->addObservation(pKF, visitedMarker->getLocalPose());

        pKF->AddMapMarker(newMapMarker);
        mpAtlas->AddMapMarker(newMapMarker);

        return newMapMarker;
    }

    void GeoSemHelpers::createMapPassage(ORB_SLAM3::Atlas *mpAtlas, ORB_SLAM3::Plane *doorPlane,
                                         ORB_SLAM3::Plane *wallPlane, bool isOpenPassage,
                                         Eigen::Vector3f passageCentroid, double width, double height)
    {
        // Variables
        ORB_SLAM3::Passage *newMapPassage = new ORB_SLAM3::Passage();
        std::vector<ORB_SLAM3::Passage *> allPassages = mpAtlas->GetAllPassages();

        // Calculate width and height of the doorway
        if (doorPlane != nullptr)
            Utils::computePlaneWidthHeight(doorPlane->getMapClouds(), width, height);

        newMapPassage->setWidth(width);
        newMapPassage->setHeight(height);
        newMapPassage->setId(allPassages.size());
        newMapPassage->setPassable(isOpenPassage);
        newMapPassage->addAssociateWall(wallPlane);
        newMapPassage->setMap(mpAtlas->GetCurrentMap());

        // If an associated door exists, use its plane equation
        if (doorPlane != nullptr)
        {
            newMapPassage->setAssociateDoor(doorPlane);
            newMapPassage->setCentroid(doorPlane->getCentroid());
            newMapPassage->setGlobalEquation(doorPlane->getGlobalEquation());
            newMapPassage->setPassageType(ORB_SLAM3::Passage::passageVariant::DOORWAY);
        }
        else
        {
            newMapPassage->setCentroid(passageCentroid);
            newMapPassage->setGlobalEquation(wallPlane->getGlobalEquation());
            newMapPassage->setPassageType(ORB_SLAM3::Passage::passageVariant::UNDEFINED);
        }

        // Check if the passage already exists
        for (const auto &existingPassage : allPassages)
        {
            // Check based on the thresholded distance between the centroids of the passages
            double distance = (newMapPassage->getCentroid() - existingPassage->getCentroid()).norm();
            if (distance < SystemParams::GetParams()->sem_seg.passage_centroid_distance_thresh)
                return;
        }

        // Otherwise, add the new passage to the map
        std::string doorStatus = isOpenPassage ? "open" : "blocked";
        std::string info = doorStatus + ", " + std::to_string(std::round(width * 10) / 10.0) +
                           "x" + std::to_string(std::round(height * 10) / 10.0) + "m";
        std::cout << "[GeoSemHelper] Creating Passage#" << newMapPassage->getId() << " ("
                  << info << ") ..." << std::endl;

        mpAtlas->AddMapPassage(newMapPassage);
    }

    ORB_SLAM3::Room *GeoSemHelpers::createBlankRoomCandidate(ORB_SLAM3::Atlas *mpAtlas,
                                                             Eigen::Vector3d centroid)
    {
        // Variable
        int roomId = mpAtlas->GetAllRooms().size();
        ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

        // Fill the room entity
        newRoom->setId(roomId);
        newRoom->setCentroid(centroid);
        newRoom->setMap(mpAtlas->GetCurrentMap());
        newRoom->setName("SE#" + std::to_string(roomId));
        newRoom->setRoomVariant(ORB_SLAM3::Room::roomVariant::UNDEFINED);

        // Add the room to the map
        // mpAtlas->AddCandidateMapRoom(newRoom);

        return newRoom;
    }

    void GeoSemHelpers::createMapRoomCandidateByMarker(Atlas *mpAtlas, ORB_SLAM3::Room *matchedRoom,
                                                       ORB_SLAM3::Marker *attachedMarker)
    {
        // Variables
        bool roomAlreadyInMap = false;

        // Check if a room has not been created before for this marker
        for (auto room : mpAtlas->GetAllMarkerBasedMapRooms())
            if (room->getMetaMarkerId() == attachedMarker->getId())
                roomAlreadyInMap = true;

        if (roomAlreadyInMap)
            return;

        // Variables
        ORB_SLAM3::Room *newMapRoomCandidate = new ORB_SLAM3::Room();

        // Fill the room entity
        newMapRoomCandidate->setHasKnownLabel(true);
        newMapRoomCandidate->setMetaMarker(attachedMarker);
        newMapRoomCandidate->setName(matchedRoom->getName());
        newMapRoomCandidate->setMap(mpAtlas->GetCurrentMap());
        newMapRoomCandidate->setId(mpAtlas->GetAllRooms().size());
        newMapRoomCandidate->setMetaMarkerId(matchedRoom->getMetaMarkerId());
        newMapRoomCandidate->setCentroid(attachedMarker->getGlobalPose().translation().cast<double>());

        std::cout
            << "- New room candidate detected: Room#" << newMapRoomCandidate->getId() << " ("
            << newMapRoomCandidate->getName() << ") using marker #" << attachedMarker->getId() << "!\n";

        mpAtlas->AddCandidateMapRoom(newMapRoomCandidate);
    }

    void GeoSemHelpers::associateGroundPlaneToRoom(Atlas *mpAtlas, ORB_SLAM3::Room *givenRoom)
    {
        std::vector<ORB_SLAM3::Plane *> allWalls = givenRoom->getWalls();
        ORB_SLAM3::Plane *associatedGroundPlane = nullptr;
        size_t maxInliers = 0;

        // get the ground planes from the Atlas
        std::vector<ORB_SLAM3::Plane *> groundPlanes;
        for (const auto &plane : mpAtlas->GetAllPlanes())
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::GROUND)
                groundPlanes.push_back(plane);

        if (groundPlanes.empty())
            // no ground planes in the Atlas
            return;
        else
        {
            // check which ground plane has the most points within the walls
            for (const auto &plane : groundPlanes)
            {
                // count inliers of the plane
                size_t inliers = countGroundPlanePointsWithinWalls(allWalls, plane);

                // update the associated ground plane if the current plane has more inliers
                if (inliers > maxInliers)
                {
                    maxInliers = inliers;
                    associatedGroundPlane = plane;
                }
            }

            if (associatedGroundPlane != nullptr)
                givenRoom->setGroundPlane(associatedGroundPlane);
            else
                // set the biggest ground plane as the ground plane of the room
                // [TODO] - logic for when ground plane is not found within the walls
                givenRoom->setGroundPlane(mpAtlas->GetBiggestGroundPlane());
        }
    }

    size_t GeoSemHelpers::countGroundPlanePointsWithinWalls(std::vector<ORB_SLAM3::Plane *> &roomWalls, ORB_SLAM3::Plane *groundPlane)
    {
        // [TODO] - verify the correctness of this function
        // the point cloud of the ground plane
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr groundCloud = groundPlane->getMapClouds();

        // the number of points within the walls
        size_t count = 0;

        // store the wall equations
        std::vector<Eigen::Vector4d> wallEquations;
        for (const auto &wall : roomWalls)
            wallEquations.push_back(wall->getGlobalEquation().coeffs());

        // for each point in the ground plane, check if it is within the walls
        for (const auto &point : groundCloud->points)
        {
            bool isWithinWalls = true;
            for (const auto &wallEquation : wallEquations)
            {
                // convert the point to Eigen vector
                Eigen::Vector3d pointVec = Eigen::Vector3d(point.x, point.y, point.z);

                // substitute the point into the wall equation to get the signed distance
                float signedDistance = wallEquation.head<3>().dot(pointVec) + wallEquation(3);

                // if the point is outside the wall, break the loop
                if (signedDistance < 0)
                {
                    isWithinWalls = false;
                    break;
                }
            }

            // if the point is within the walls, increment the count
            if (isWithinWalls)
                count++;
        }
        return count;
    }

    void GeoSemHelpers::createMapFloor(ORB_SLAM3::Atlas *mpAtlas)
    {
        // Create a new floor object
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        ORB_SLAM3::Floor *newMapFloor = new ORB_SLAM3::Floor();

        // Variables
        int floorId = mpAtlas->GetAllFloors().size();

        // Fill the floor entity
        newMapFloor->setOpId(-1);
        newMapFloor->setOpIdG(-1);
        newMapFloor->setId(floorId);
        newMapFloor->setCentroid(centroid);
        newMapFloor->setMap(mpAtlas->GetCurrentMap());
        newMapFloor->setName("Floor#" + std::to_string(floorId));

        // Add the floor to the map
        mpAtlas->AddMapFloor(newMapFloor);

        std::cout << "[GeoSemHelper] Creating Floor#" << newMapFloor->getId() << " ..." << std::endl;
    }
}