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

#include "SemanticsManager.h"

namespace ORB_SLAM3
{
    SemanticsManager::SemanticsManager(Atlas *pAtlas)
    {
        mpAtlas = pAtlas;

        // Get the system parameters
        sysParams = SystemParams::GetParams();
    }

    void SemanticsManager::Run()
    {
        while (true)
        {
            // get the start time
            auto start = std::chrono::high_resolution_clock::now();

            // Update the ground plane, as it might have been updated
            // even when semantic segmentation did not detect any planes
            Plane *mainGroundPlane = mpAtlas->GetBiggestGroundPlane();
            if (mainGroundPlane != nullptr)
            {
                // Re-compute the transformation from ground to horizontal - maybe global eq. changed
                mPlanePoseMat = computePlaneToHorizontal(mainGroundPlane);

                // Filter planes with semantics
                filterGroundPlanes(mainGroundPlane);
                filterWallPlanes();
            }

            // Re-associate semantic planes if they get close to each other :)) after optimization
            if (sysParams->sem_seg.reassociate.enabled)
                Utils::reAssociateSemanticPlanes(mpAtlas);

            // Detect passages?
            if (sysParams->sem_seg.enable_passage_detection)
            {
                detectDoorsAndDoorways(mpAtlas);
                updatePassages(mpAtlas);
            }

            // Check for possible room candidates
            if (sysParams->room_seg.method == SystemParams::room_seg::Method::FREE_SPACE)
                detectRoom_FreeSpaceCluster();
            else if (sysParams->room_seg.method == SystemParams::room_seg::Method::GNN)
                detectRoom_GNN();

            // Re-associate rooms based on walls and clusters
            reAssociateRooms();

            // Check detected floors
            getUpdatedFloors();

            // wait until its intervalTime to run the next iteration
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            if (elapsed.count() < runInterval)
                std::this_thread::sleep_for(std::chrono::seconds(runInterval) - elapsed);
        }
    }

    std::vector<std::vector<Eigen::Vector3d>> SemanticsManager::getLatestSkeletonCluster()
    {
        unique_lock<std::mutex> lock(mMutexNewRooms);
        // Get the latest skeleton cluster from Atlas
        return mpAtlas->GetSkeletoClusterPoints();
    }

    std::vector<ORB_SLAM3::Room *> SemanticsManager::getLatestGNNRoomCandidates()
    {
        // [TODO]
    }

    void SemanticsManager::filterWallPlanes()
    {
        for (const auto &plane : mpAtlas->GetAllPlanes())
        {
            if (plane->getExpectedPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
            {
                // wall validation based on the mPlanePoseMat
                // only works if the ground plane is set, needs the correction matrix: mPlanePoseMat
                Eigen::Vector3f transformedPlaneCoefficients = transformPlaneEqToGroundReference(plane->getGlobalEquation().coeffs());

                // if the transformed plane is vertical based on absolute value, then assign semantic, otherwise ignore
                // threshold should be leniently set (ideally with correct ground plane reference, this value should be close to 0.00)
                if (abs(transformedPlaneCoefficients(1)) > sysParams->sem_seg.max_tilt_wall)
                    plane->resetPlaneSemantics();
            }
        }
    }

    void SemanticsManager::filterGroundPlanes(Plane *groundPlane)
    {
        // discard ground planes that have height above a threshold from the biggest ground plane
        // [TODO] - Whether to use biggest ground plane or lowest ground plane?

        // get the median height of the plane to compute the threshold
        float threshY = computeGroundPlaneHeight(groundPlane) - sysParams->sem_seg.max_step_elevation;

        // go through all ground planes to check validity
        int groundPlaneId = groundPlane->getId();
        for (const auto &plane : mpAtlas->GetAllPlanes())
        {
            if (plane->getExpectedPlaneType() != ORB_SLAM3::Plane::planeVariant::GROUND || plane->getId() == groundPlaneId)
                continue;

            // if the plane is above the threshold (inverted y), then reset the plane semantics
            if (computeGroundPlaneHeight(plane) < threshY)
            {
                plane->resetPlaneSemantics();
                continue;
            }

            // filter here based on orientation of the plane (needs to be horizontal)
            Eigen::Vector3f transformedPlaneCoefficients = transformPlaneEqToGroundReference(plane->getGlobalEquation().coeffs());

            // if the transformed plane is horizontal based on absolute value, then assign semantic, otherwise ignore
            // threshold should be leniently set (ideally with correct ground plane reference, this value should be close to 0.00)
            if (abs(transformedPlaneCoefficients(0)) > sysParams->sem_seg.max_tilt_ground)
                plane->resetPlaneSemantics();
        }
    }

    void SemanticsManager::detectDoorsAndDoorways(ORB_SLAM3::Atlas *pAtlas)
    {
        // Variables
        int windowSize = sysParams->sem_seg.passage_kf_window;

        // Get all planes and filter only DOOR and WALL variants
        std::vector<ORB_SLAM3::Plane *> allPlanes = pAtlas->GetAllPlanes();
        std::vector<ORB_SLAM3::Plane *> wallPlanes;
        std::vector<ORB_SLAM3::Plane *> doorPlanes;
        for (const auto &plane : allPlanes)
        {
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
                wallPlanes.push_back(plane);
            else if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::DOOR)
                doorPlanes.push_back(plane);
        }

        // Procedure#1 - Detecting closed doors (blocked passages)
        // Detect doors with the same normal as walls and close to walls
        for (const auto &door : doorPlanes)
        {
            for (const auto &wall : wallPlanes)
            {
                // Skip if the door and wall are not parallel
                if (!Utils::arePlanesParallel(door, wall))
                    continue;

                // Skip if the door and wall are facing each other
                if (Utils::arePlanesFacingEachOther(door, wall))
                    continue;

                // Skip if the door and wall are not close to each other
                if (Utils::arePlanesApartEnough(door, wall, sysParams->sem_seg.max_wall_door_distance))
                    continue;

                // Otherwise, it is a valid closed door to be connected to the wall
                GeoSemHelpers::createMapPassage(mpAtlas, door, wall, false);
            }
        }

        // Procedure#2 - Detecting open passages by checking if trajectory crosses the wall plane
        std::vector<ORB_SLAM3::KeyFrame *> allKFs = pAtlas->GetAllKeyFrames();
        if (allKFs.size() < 2)
            return;

        std::sort(allKFs.begin(), allKFs.end(),
                  [](const ORB_SLAM3::KeyFrame *a, const ORB_SLAM3::KeyFrame *b)
                  {
                      return a->mnId < b->mnId;
                  });

        for (const auto &wall : wallPlanes)
        {
            const Eigen::Vector4d wallEq =
                Utils::correctPlaneDirection(wall->getGlobalEquation().coeffs());
            const Eigen::Vector3d wallNormal = wallEq.head<3>();

            // Compute signed distances for every keyframe against this wall
            struct KFRecord
            {
                ORB_SLAM3::KeyFrame *kf;
                Eigen::Vector3d posW;
                double signedDist;
            };

            std::vector<KFRecord> records;
            records.reserve(allKFs.size());

            for (auto *kf : allKFs)
            {
                if (!kf || kf->isBad())
                    continue;

                const Eigen::Vector3d posW = kf->GetPoseInverse().translation().cast<double>();

                // Only consider keyframes that are reasonably close to the wall
                const double distToPlane = std::abs(wallNormal.dot(posW) + wallEq(3));
                if (distToPlane > sysParams->sem_seg.max_kf_passage_distance)
                    continue;

                records.push_back({kf, posW, wallNormal.dot(posW) + wallEq(3)});
            }

            if (records.size() < 2)
                continue;

            // Slide a window of size N over the filtered keyframe records
            bool passageDetected = false;
            Eigen::Vector3f passageCentroid = Eigen::Vector3f::Zero();
            const int n = static_cast<int>(records.size());

            for (int start = 0; start <= n - 2 && !passageDetected; ++start)
            {
                const int end = std::min(start + windowSize - 1, n - 1);

                for (int i = start; i < end && !passageDetected; ++i)
                {
                    const KFRecord &recA = records[i];
                    const KFRecord &recB = records[i + 1];

                    // Baseline check: ignore nearly-static pairs.
                    const double baseline = (recB.posW - recA.posW).norm();
                    if (baseline < 0.10)
                        continue;

                    const double dA = recA.signedDist;
                    const double dB = recB.signedDist;

                    // Sign change → the segment crosses the plane.
                    if (dA * dB < 0.0)
                    {
                        // Compute crossing point
                        const double denom = dA - dB;
                        Eigen::Vector3f crossingPoint;

                        if (std::abs(denom) > 1e-6)
                        {
                            const double t = dA / denom;
                            const Eigen::Vector3d crossing = recA.posW + t * (recB.posW - recA.posW);
                            crossingPoint = crossing.cast<float>();
                        }
                        else
                            crossingPoint = ((recA.posW + recB.posW) * 0.5).cast<float>();

                        passageDetected = true;
                        passageCentroid = crossingPoint;
                    }
                }
            }

            if (passageDetected)
            {
                // If there is a door close to the passage centroid, then it is an open doorway
                ORB_SLAM3::Plane *potentialDoor = nullptr;

                //     // for (const auto &door : doorPlanes)
                //     //     if (Utils::calculateDistancePointToPlane(door->getGlobalEquation().coeffs(),
                //     //                                              passageCentroid.cast<double>()) <
                //     //         sysParams->sem_seg.max_wall_door_distance)
                //     //     {
                //     //         potentialDoor = door;
                //     //         break;
                //     //     }

                GeoSemHelpers::createMapPassage(mpAtlas, potentialDoor, wall, true, passageCentroid);
            }
        }
    }

    void SemanticsManager::updatePassages(ORB_SLAM3::Atlas *pAtlas)
    {
        // Get the ground plane
        ORB_SLAM3::Plane *groundPlane = pAtlas->GetBiggestGroundPlane();
        if (groundPlane == nullptr)
            return;

        // Get all passages and update their global pose to be consistent with the ground plane
        std::vector<ORB_SLAM3::Passage *> allPassages = pAtlas->GetAllPassages();

        for (const auto &passage : allPassages)
        {
            // Get the passage variant (doorway or undefined)
            ORB_SLAM3::Passage::passageVariant variant = passage->getPassageType();

            // Updating the dimensions of the passage based on the associated door plane
            ORB_SLAM3::Plane *doorPlane = passage->getAssociateDoor();

            // Blocked passages (closed doors) should be aligned with the ground plane normal
            if (!passage->isPassable())
            {
                if (doorPlane == nullptr)
                    continue;
                std::pair<double, double> widthHeight = Utils::computePlaneWidthHeight(doorPlane->getMapClouds());
                cout << "[SemanticsManager] Updating passage " << passage->getId() << " with associated door plane " << doorPlane->getId()
                     << " dimensions: width = " << widthHeight.first << "m, height = " << widthHeight.second << "m.\n";
                passage->setCentroid(doorPlane->getCentroid());
                passage->setGlobalEquation(doorPlane->getGlobalEquation());
                passage->setWidth(widthHeight.first);
                passage->setHeight(widthHeight.second);
            }
            else
            {
                ORB_SLAM3::Plane passagePlane;
                passagePlane.setGlobalEquation(passage->getGlobalEquation());
                if (!Utils::arePlanesPerpendicular(&passagePlane, groundPlane))
                {
                    // Project the passage normal onto the horizontal plane to remove tilt
                    const Eigen::Vector3d groundNormal =
                        groundPlane->getGlobalEquation().coeffs().head<3>().normalized();
                    Eigen::Vector4d globalEq = passage->getGlobalEquation().coeffs();
                    Eigen::Vector3d passageNormal = globalEq.head<3>().normalized();

                    Eigen::Vector3d correctedNormal =
                        (passageNormal - passageNormal.dot(groundNormal) * groundNormal).normalized();
                    if (correctedNormal.norm() < 1e-6)
                        continue;
                    correctedNormal.normalize();

                    // Recompute d so the plane still passes through the centroid
                    const Eigen::Vector3d centroid = passage->getCentroid().cast<double>();
                    const double d = -correctedNormal.dot(centroid);

                    Eigen::Vector4d correctedCoeffs;
                    correctedCoeffs.head<3>() = correctedNormal;
                    correctedCoeffs(3) = d;

                    passage->setGlobalEquation(g2o::Plane3D(correctedCoeffs));
                }
            }
        }
    }

    Eigen::Vector3f SemanticsManager::transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq)
    {
        // extract the rotation matrix from the transformation matrix
        Eigen::Matrix3f rotationMatrix = mPlanePoseMat.block<3, 3>(0, 0);

        // Compute the inverse transpose of the rotation matrix
        Eigen::Matrix3f inverseTransposeRotationMatrix = rotationMatrix.inverse().transpose();

        // Transform the coefficients of the plane equation
        Eigen::Vector3f transformedPlaneCoefficients = inverseTransposeRotationMatrix * planeEq.head<3>().cast<float>();
        transformedPlaneCoefficients.normalize();

        return transformedPlaneCoefficients;
    }

    float SemanticsManager::computeGroundPlaneHeight(Plane *groundPlane)
    {
        // transform the planeCloud according to the planePose
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = groundPlane->getMapClouds();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::transformPointCloud(*planeCloud, *transformedCloud, mPlanePoseMat);

        // get the median height of the plane
        std::vector<float> yVals;
        for (const auto &point : transformedCloud->points)
        {
            yVals.push_back(point.y);
        }
        size_t numPoint = yVals.size() / 2;
        std::partial_sort(yVals.begin(), yVals.begin() + numPoint, yVals.end(), std::greater<float>());
        return yVals[numPoint - 1];
    }

    Eigen::Matrix4f SemanticsManager::computePlaneToHorizontal(const Plane *plane)
    {
        // initialize the transformation with translation set to a zero vector
        Eigen::Isometry3d planePose;
        planePose.translation() = Eigen::Vector3d(0, 0, 0);

        // normalize the normal vector
        Eigen::Vector3d normal = plane->getGlobalEquation().coeffs().head<3>();

        // get the rotation from the ground plane to the plane with y-facing vertical downwards
        Eigen::Vector3d verticalAxis = Eigen::Vector3d(0, -1, 0);
        Eigen::Quaterniond q;
        q.setFromTwoVectors(normal, verticalAxis);
        planePose.linear() = q.toRotationMatrix();

        // form homogenous transformation matrix
        Eigen::Matrix4f planePoseMat = planePose.matrix().cast<float>();
        planePoseMat(3, 3) = 1.0;

        return planePoseMat;
    }

    void SemanticsManager::detectRoom_FreeSpaceCluster()
    {
        // Variables
        std::vector<ORB_SLAM3::Plane *> allWalls;

        // Get the skeleton clusters
        std::vector<std::vector<Eigen::Vector3d>> clusters = getLatestSkeletonCluster();

        // If there are no clusters, return
        if (clusters.empty())
            return;

        // Get all the mapped planes
        std::vector<ORB_SLAM3::Plane *> allPlanes = mpAtlas->GetAllPlanes();

        // Filter the planes to get only the walls
        for (const auto &plane : allPlanes)
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
                allWalls.push_back(plane);

        // Find the walls within the threshold distance to the cluster points
        for (const auto &cluster : clusters)
        {
            // If the cluster is empty, continue to the next cluster
            if (cluster.empty())
                continue;

            // Initializations
            std::vector<ORB_SLAM3::Plane *> closestWalls;

            // Calculate the cluster centroid
            Eigen::Vector3d clusterCentroid = Utils::computeCentroidFromPoints(cluster);

            for (const auto &wall : allWalls)
            {
                int closeCount = 0;
                // Check the distance between the cluster centroid and the wall centroid
                const double centroidDistance = (wall->getCentroid().cast<double>() - clusterCentroid).norm();
                if (centroidDistance < sysParams->room_seg.cluster_centroid_wall_centroid_distance_thresh)
                {
                    // Check the minimum wall to all cluster points distance, and only add if that minimum is below the threshold
                    for (const auto &point : cluster)
                    {
                        const double distance = Utils::calculateDistancePointToPlane(wall->getGlobalEquation().coeffs(), point);
                        if (distance < sysParams->room_seg.cluster_point_wall_distance_thresh)
                            closeCount++;
                    }
                    // Require at least 30% of cluster points to be close to the wall
                    if (closeCount >= static_cast<int>(0.3 * cluster.size()))
                        closestWalls.push_back(wall);
                }
            }

            // Filter out closest walls with normals pointing away from the cluster
            closestWalls.erase(
                std::remove_if(closestWalls.begin(), closestWalls.end(),
                               [&](ORB_SLAM3::Plane *wall)
                               {
                                   // Plane normal from global equation
                                   Eigen::Vector3d normal = wall->getGlobalEquation().normal().cast<double>();
                                   // Direction from cluster centroid to wall centroid
                                   Eigen::Vector3d direction = (wall->getCentroid().cast<double>() - clusterCentroid).normalized();
                                   // Remove walls whose normal points away from the cluster centroid
                                   return normal.dot(direction) >= 0;
                               }),
                closestWalls.end());

            // Filter out closest walls that are already assigned to other rooms
            // Note: this is done by checking GetRoomWallPlaneById unordered-map in Atlas
            closestWalls.erase(
                std::remove_if(closestWalls.begin(), closestWalls.end(),
                               [&](ORB_SLAM3::Plane *wall)
                               {
                                   return mpAtlas->GetRoomWallPlaneById(wall->getId()) != nullptr;
                               }),
                closestWalls.end());

            // Remove duplicate walls
            std::sort(closestWalls.begin(), closestWalls.end());
            closestWalls.erase(std::unique(closestWalls.begin(), closestWalls.end()), closestWalls.end());

            // If no closestWalls found, continue to the next cluster
            if (closestWalls.empty())
                continue;

            // Create a new room candidate for the cluster
            ORB_SLAM3::Room *newRoom = GeoSemHelpers::createBlankRoomCandidate(mpAtlas, clusterCentroid);

            // Now, check if a room already exists in the map for this cluster
            ORB_SLAM3::Room *existedRoom = associateRooms(clusterCentroid, closestWalls);
            if (existedRoom == nullptr)
            {
                // Add the new room to the map
                mpAtlas->AddCandidateMapRoom(newRoom);
                std::cout
                    << "[SemMgr] New room candidate created: SE#" << newRoom->getId() << "..." << std::endl;
                // Set the new room as the existedRoom for further processing
                existedRoom = newRoom;
            }

            // Get the room's walls
            std::vector<ORB_SLAM3::Plane *> roomWalls = existedRoom->getWalls();

            // If the walls close to the cluster and facing the cluster are not already in the room, add them to the room
            for (const auto &wall : closestWalls)
            {
                // If the wall is not already in closestWalls, add it to the room
                if (std::find(roomWalls.begin(), roomWalls.end(), wall) == roomWalls.end())
                {
                    existedRoom->setWalls(wall);
                    // Keep track of the wall-to-room association in the Atlas
                    mpAtlas->AddRoomWallPlane(wall);
                }
            }

            // Check room layout
            // [Hint] the free-space based room detection creates ONLY rooms and not corridors, due to the lack of definition
            if (existedRoom->getRoomVariant() == Room::UNDEFINED)
            {
                // If less than 2 walls, keep it undefined
                if (roomWalls.size() >= 2)
                {
                    // Get pairs of facing planes
                    std::vector<std::pair<ORB_SLAM3::Plane *, ORB_SLAM3::Plane *>> facingWalls =
                        Utils::getFacingPlanes(roomWalls);
                    // If there is at least one pair of facing walls, classify as room
                    if (!facingWalls.empty())
                    {
                        existedRoom->setRoomVariant(ORB_SLAM3::Room::ROOM);
                        existedRoom->setName("Room#" + std::to_string(existedRoom->getId()));
                        std::cout << "[SemMgr] Structural Element #" << existedRoom->getId() << " got classified as a Room." << std::endl;
                    }
                }
            }

            /**
             * 🚧 [vS-Graphs v.1.5] Archived room/corridor classification based on wall count and perpendicularity
             */
            // if (facingWalls.size() == 1)
            // {
            //     existedRoom->setRoomVariant(ORB_SLAM3::Room::CORRIDOR);
            //     existedRoom->setName("Corridor#" + std::to_string(existedRoom->getId()));
            //     std::cout << "[SemMgr] Structural Element #" << existedRoom->getId() << " got classified as a Corridor." << std::endl;
            // }
            // else
            // {
            //     // Variables
            //     bool isRoom = false;

            //     // Iterate through each pair of facing walls
            //     for (size_t idx1 = 0; idx1 < facingWalls.size(); ++idx1)
            //     {
            //         for (size_t idx2 = idx1 + 1; idx2 < facingWalls.size(); ++idx2)
            //         {
            //             // Variables
            //             int perpCount = 0;

            //             // Get the walls
            //             ORB_SLAM3::Plane *wall1P1 = facingWalls[idx1].first;
            //             ORB_SLAM3::Plane *wall2P1 = facingWalls[idx1].second;
            //             ORB_SLAM3::Plane *wall1P2 = facingWalls[idx2].first;
            //             ORB_SLAM3::Plane *wall2P2 = facingWalls[idx2].second;

            //             // Check if wall pairs form a square, considering the perpendicularity threshold
            //             if (Utils::arePlanesPerpendicular(wall1P1, wall1P2))
            //                 perpCount++;
            //             if (Utils::arePlanesPerpendicular(wall1P1, wall2P2))
            //                 perpCount++;
            //             if (Utils::arePlanesPerpendicular(wall2P1, wall1P2))
            //                 perpCount++;
            //             if (Utils::arePlanesPerpendicular(wall2P1, wall2P2))
            //                 perpCount++;

            //             if (perpCount >= 2)
            //             {
            //                 isRoom = true;
            //                 break;
            //             }
            //         }
            //         if (isRoom)
            //             break;
            //     }

            //     // If the flag is true, it is a room
            //     if (isRoom)
            //     {
            //         existedRoom->setRoomVariant(ORB_SLAM3::Room::ROOM);
            //         existedRoom->setName("Room#" + std::to_string(existedRoom->getId()));
            //         std::cout << "[SemMgr] Structural Element #" << existedRoom->getId() << " got classified as a Room." << std::endl;
            //     }
            // }
        }
    }

    void SemanticsManager::detectRoom_GNN()
    {
        // [TODO] Needs to be implemented
    }

    void SemanticsManager::getUpdatedFloors()
    {
        // [TODO] The current version supports singe floor only.
        if (mpAtlas->GetAllFloors().size() < 1)
            // Create a new floor object
            GeoSemHelpers::createMapFloor(mpAtlas);
        else
        {
            // Update the existing floor object to cotain all rooms
            std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

            // If the room is bad, remove it from the list
            allRooms.erase(
                std::remove_if(allRooms.begin(), allRooms.end(),
                               [](ORB_SLAM3::Room *room)
                               { return room->isBad(); }),
                allRooms.end());

            // Get the centroid of each room
            std::vector<Eigen::Vector3d> roomCentroids;
            for (auto &room : allRooms)
                // Add the centroid to the vector
                roomCentroids.push_back(room->getCentroid());

            // Set all the detected rooms to the floor (assuming single floor)
            for (auto &floor : mpAtlas->GetAllFloors())
            {
                // Connect all rooms to the floor
                floor->setRooms(allRooms);
                // Update the floor centroid
                floor->setCentroid(Utils::computeCentroidFromPoints(roomCentroids));
            }
        }
    }

    ORB_SLAM3::Room *SemanticsManager::associateRooms(const Eigen::Vector3d clusterCentroid,
                                                      const std::vector<ORB_SLAM3::Plane *> &wallList)
    {
        // Variables
        size_t maxMatches = 0;
        ORB_SLAM3::Room *bestMatch = nullptr;
        std::vector<ORB_SLAM3::Room *> potentialMatches;
        double distanceThresh = sysParams->room_seg.center_distance_thresh;

        // Get all the rooms in the map
        const auto &allRooms = mpAtlas->GetAllRooms();
        if (allRooms.empty())
            return nullptr;

        // Collect potential matches (rooms within threshold)
        for (const auto &mapRoom : allRooms)
        {
            // Calculate distance between centroids
            double distance = (clusterCentroid - mapRoom->getCentroid()).norm();
            // Check if the distance is within the threshold
            if (distance < distanceThresh)
                potentialMatches.push_back(mapRoom);
        }

        // Among potential matches, choose the one with most wall overlaps
        for (const auto &mapRoom : potentialMatches)
        {
            size_t matches = 0;
            // Get the walls of the map room
            std::vector<ORB_SLAM3::Plane *> mapRoomWalls = mapRoom->getWalls();

            if (mapRoomWalls.empty())
                continue;

            // Use a hash set for faster lookup
            std::unordered_set<int> wallIds;
            wallIds.reserve(mapRoomWalls.size());
            for (auto *wall : mapRoomWalls)
                wallIds.insert(wall->getId());

            // Wall matching
            for (auto *wall : wallList)
                if (wallIds.count(wall->getId()))
                    matches++;

            // Update the best match if the current room has more matches
            if (matches > maxMatches)
            {
                maxMatches = matches;
                bestMatch = mapRoom;
            }
        }

        // A minimum number of matches
        return (maxMatches > 0) ? bestMatch : nullptr;
    }

    void SemanticsManager::reAssociateRooms()
    {
        // Variables
        double distanceThresh = sysParams->room_seg.center_distance_thresh;
        auto allRooms = mpAtlas->GetAllRooms();

        // Loop over all rooms to find duplicates
        for (size_t i = 0; i < allRooms.size(); ++i)
        {
            ORB_SLAM3::Room *room1 = allRooms[i];
            if (!room1 || room1->isBad())
                continue;

            for (size_t j = i + 1; j < allRooms.size(); ++j)
            {
                ORB_SLAM3::Room *room2 = allRooms[j];
                if (!room2 || room2->isBad())
                    continue;

                double distance = (room1->getCentroid() - room2->getCentroid()).norm();
                if (distance < distanceThresh)
                {
                    // Take all walls from room2 that are not already in room1
                    for (auto *wall : room2->getWalls())
                    {
                        bool exists = false;
                        for (auto *w : room1->getWalls())
                        {
                            if (wall->getId() == w->getId())
                            {
                                exists = true;
                                break;
                            }
                        }
                        if (!exists)
                            room1->setWalls(wall);
                    }

                    // Merge room2 into room1
                    room2->setBad();
                    room1->setRoomVariant(ORB_SLAM3::Room::ROOM);
                    room1->setName("Room#" + std::to_string(room1->getId()));

                    std::cout << "[SemMgr] Merging Room #" << room2->getId()
                              << " into Room #" << room1->getId()
                              << " due to proximity." << std::endl;
                }
            }
        }
    }
}