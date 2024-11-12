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

            // Check for possible room candidates
            // if (sysParams->room_seg.method == SystemParams::room_seg::Method::GEOMETRIC)
            // updateMapRoomCandidateToRoomGeo(thisKF);
            if (sysParams->room_seg.method == SystemParams::room_seg::Method::FREE_SPACE)
                detectMapRoomCandidateVoxblox();

            // // count non-bad map points
            // int numMapPoints = 0;
            // for (const auto &mp: mpAtlas->GetAllMapPoints())
            //     if (mp && !mp->isBad())
            //         numMapPoints++;
            // std::cout << "[Semantics Manager] Number of non-bad map points: " << numMapPoints << std::endl;

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

    bool SemanticsManager::getRectangularRoom(
        std::pair<std::pair<Plane *, Plane *>, std::pair<Plane *, Plane *>> &givenRoom,
        const std::vector<std::pair<Plane *, Plane *>> &facingWalls,
        double perpThreshDeg)
    {
        // Convert threshold from degrees to radians
        double perpThreshold = perpThreshDeg * Utils::DEG_TO_RAD;

        // Iterate through each pair of facing walls
        for (size_t idx1 = 0; idx1 < facingWalls.size(); ++idx1)
            for (size_t idx2 = idx1 + 1; idx2 < facingWalls.size(); ++idx2)
            {
                // Get the walls
                Plane *wall1P1 = facingWalls[idx1].first;
                Plane *wall2P1 = facingWalls[idx1].second;
                Plane *wall1P2 = facingWalls[idx2].first;
                Plane *wall2P2 = facingWalls[idx2].second;

                // Check if wall pairs form a square, considering the perpendicularity threshold
                if (Utils::arePlanesPerpendicular(wall1P1, wall1P2, perpThreshold) &&
                    Utils::arePlanesPerpendicular(wall1P1, wall2P2, perpThreshold) &&
                    Utils::arePlanesPerpendicular(wall2P1, wall1P2, perpThreshold) &&
                    Utils::arePlanesPerpendicular(wall2P1, wall2P2, perpThreshold))
                {
                    givenRoom = std::make_pair(facingWalls[idx1], facingWalls[idx2]);
                    return true;
                }
            }
        // No rectangular room found
        return false;
    }

    /**
     * 🚧 [vS-Graphs v.2.0] This solution is not very reliable.
     * It is highly recommended to use the Skeleton Voxblox version.
     */
    void SemanticsManager::updateMapRoomCandidateToRoomGeo(KeyFrame *pKF)
    {
        // Get all the mapped planes and rooms
        std::vector<Room *> allRooms = mpAtlas->GetAllRooms();
        std::vector<Plane *> allPlanes = mpAtlas->GetAllPlanes();

        // Filter the planes to get only the walls
        std::vector<Plane *> allWalls;
        for (auto plane : allPlanes)
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
                allWalls.push_back(plane);

        // Get the closest walls to the current KeyFrame
        std::vector<Plane *> closestWalls;
        for (auto wall : allWalls)
        {
            // Calculate distance between wall centroid and KeyFrame pose
            double distance = Utils::calculateDistancePointToPlane(wall->getGlobalEquation().coeffs(),
                                                                   pKF->GetPose().translation().cast<double>());

            // Update closestWalls if distance is smaller than the threshold
            if (distance < sysParams->room_seg.marker_wall_distance_thresh)
                closestWalls.push_back(wall);
        }

        // Get all the facing walls
        std::vector<std::pair<Plane *, Plane *>> facingWalls =
            Utils::getAllPlanesFacingEachOther(closestWalls);

        // If there is at least one pair of facing wall
        if (facingWalls.size() > 0)
            // Loop over all the rooms
            for (auto roomCandidate : allRooms)
            {
                // Fetch parameters of the room candidate
                roomCandidate->setHasKnownLabel(true);
                Sophus::SE3f metaMarkerPose = roomCandidate->getMetaMarker()->getGlobalPose();

                // Find the closest facing walls to the room center
                std::pair<Plane *, Plane *> closestPair1, closestPair2;
                double minDistance1 = std::numeric_limits<double>::max();
                double minDistance2 = std::numeric_limits<double>::max();

                for (auto facingWallsPair : facingWalls)
                {
                    // Calculate distance between wall centroids and metaMarkerPose
                    double distance1 = Utils::calculateDistancePointToPlane(facingWallsPair.first->getGlobalEquation().coeffs(),
                                                                            metaMarkerPose.translation().cast<double>());
                    double distance2 = Utils::calculateDistancePointToPlane(facingWallsPair.second->getGlobalEquation().coeffs(),
                                                                            metaMarkerPose.translation().cast<double>());

                    // Update closestPair1 if distance1 is smaller
                    if (distance1 < minDistance1)
                    {
                        minDistance1 = distance1;
                        closestPair1 = facingWallsPair;
                    }

                    // Update closestPair2 if distance2 is smaller and it's not the same facingWallsPair as closestPair1
                    if (distance2 < minDistance2 && facingWallsPair != closestPair1)
                    {
                        minDistance2 = distance2;
                        closestPair2 = facingWallsPair;
                    }
                }

                // If the room is a corridor
                if (roomCandidate->getIsCorridor())
                {
                    if (closestPair1.first != nullptr && closestPair1.second != nullptr)
                    {
                        // Update the room walls
                        roomCandidate->setWalls(closestPair1.first);
                        roomCandidate->setWalls(closestPair1.second);
                    }
                }
                else
                {
                    // Update the room walls
                    if (closestPair1.first != nullptr && closestPair1.second != nullptr)
                    {
                        roomCandidate->setWalls(closestPair1.first);
                        roomCandidate->setWalls(closestPair1.second);
                    }
                    if (closestPair2.first != nullptr && closestPair2.second != nullptr)
                    {
                        roomCandidate->setWalls(closestPair2.first);
                        roomCandidate->setWalls(closestPair2.second);
                    }
                }

                // [TODO] Check the isCorridor and the number of walls we connected
                // If it is more than 4 four 4-wall room or 2 for corridor, we should take only the ones closest to the room
            }
    }

    void SemanticsManager::detectMapRoomCandidateVoxblox()
    {
        // Variables
        std::vector<Plane *> allWalls;
        ORB_SLAM3::Room *newClusterBasedRoom = nullptr;
        double minValidSpace = sysParams->room_seg.min_wall_distance_thresh;

        // Get the skeleton clusters
        std::vector<std::vector<Eigen::Vector3d>> clusters = getLatestSkeletonCluster();

        // Get all the mapped planes
        std::vector<Plane *> allPlanes = mpAtlas->GetAllPlanes();

        // Filter the planes to get only the walls
        for (const auto &plane : allPlanes)
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
                allWalls.push_back(plane);

        // Find the walls within the threshold distance to the cluster points
        int counter = 0;
        for (const auto &cluster : clusters)
        {
            // Initializations
            std::vector<Plane *> closestWalls;
            std::pair<std::pair<Plane *, Plane *>, std::pair<Plane *, Plane *>> rectangularRoom;

            // Loop over all walls
            for (const auto &wall : allWalls)
            {
                // Loop over all cluster points
                for (const auto &point : cluster)
                {
                    // Calculate distance between wall centroid and cluster centroid
                    const double distance = Utils::calculateDistancePointToPlane(wall->getGlobalEquation().coeffs(), point);
                    // If the distance is smaller than the threshold, add the wall to closestWalls
                    if (distance < sysParams->room_seg.cluster_point_wall_distance_thresh)
                    {
                        // Add the wall to closestWalls
                        closestWalls.push_back(wall);
                        break;
                    }
                }
            }

            // Print the closest walls
            std::cout << "\nCluster#" << counter << std::endl;
            for (const auto &wall : closestWalls)
                std::cout << "- Wall#" << wall->getId() << " is close enough ..." << std::endl;
            counter++;

            // If there is only one wall, no need to check for a room/corridor
            if (closestWalls.size() < 2)
                continue;

            // Get all the facing walls
            std::vector<std::pair<Plane *, Plane *>> facingWalls =
                Utils::getAllPlanesFacingEachOther(closestWalls);

            // Print the facing walls
            for (const auto &facingWall : facingWalls)
                std::cout << "- Facing walls: " << facingWall.first->getId() << " & " << facingWall.second->getId() << std::endl;

            // If no facing walls are found, continue to the next cluster
            if (facingWalls.size() == 0)
                continue;

            // Check wall conditions if they shape a square room (with perpendicularity threshold)
            bool isRectRoomFound = getRectangularRoom(rectangularRoom, facingWalls,
                                                      sysParams->room_seg.walls_perpendicularity_thresh);

            // Create room candidates for them
            if (isRectRoomFound)
            {
                std::vector<ORB_SLAM3::Plane *> walls = {rectangularRoom.first.first, rectangularRoom.first.second,
                                                         rectangularRoom.second.first, rectangularRoom.second.second};
                // Search for a room candidate within the given range (marker-based)
                // If found, augment information from new room to that one
                newClusterBasedRoom = GeoSemHelpers::createMapRoomCandidateByFreeSpace(mpAtlas, false, walls);
            }
            else
            {
                // [TODO] What if we have more than one facing wall?
                // Get the walls
                std::vector<ORB_SLAM3::Plane *> walls = {facingWalls[0].first, facingWalls[0].second};
                // Check if the walls are apart beyond a threshold
                if (!Utils::arePlanesApartEnough(walls[0], walls[1], minValidSpace))
                    continue;
                // Create a corridor
                newClusterBasedRoom = GeoSemHelpers::createMapRoomCandidateByFreeSpace(mpAtlas, true, walls,
                                                                                       Utils::getClusterCenteroid(cluster));
            }

            // Check if the room has not been created before
            ORB_SLAM3::Room *foundMarkerBasedRoom = roomAssociation(newClusterBasedRoom, mpAtlas->GetAllMarkerBasedMapRooms());
            if (foundMarkerBasedRoom != nullptr)
            {
                // If the room already exists, update the existing room candidate with the new information
                GeoSemHelpers::augmentMapRoomCandidate(foundMarkerBasedRoom, newClusterBasedRoom, true);
            }
            else
            {
                // Otherwise, add the new room candidate to the map
                ORB_SLAM3::Room *foundDetectedRoom = roomAssociation(newClusterBasedRoom, mpAtlas->GetAllDetectedMapRooms());
                if (foundDetectedRoom == nullptr)
                {
                    std::cout << "\n[SemSeg]" << std::endl;
                    std::cout
                        << "- New free-space cluster room detected! Mapping Room#" << newClusterBasedRoom->getId()
                        << "!" << std::endl;
                    mpAtlas->AddDetectedMapRoom(newClusterBasedRoom);
                }
            }
        }
    }

    void SemanticsManager::detectMapRoomCandidateGNN()
    {
        // [TODO] Needs to be implemented
    }

    ORB_SLAM3::Room *SemanticsManager::roomAssociation(const ORB_SLAM3::Room *givenRoom,
                                                       const vector<Room *> &givenRoomList)
    {
        // Variables
        ORB_SLAM3::Room *foundMappedRoom = nullptr;
        double minDistance = sysParams->room_seg.center_distance_thresh;

        if (givenRoomList.empty())
            return nullptr;

        // Get the given room center
        Eigen::Vector3d detetedRoomCenter = givenRoom->getRoomCenter();

        // Check to find the room with the minimum distance from the center
        for (const auto &mapRoom : givenRoomList)
        {
            Eigen::Vector3d mapRoomCenter = mapRoom->getRoomCenter();
            double distance = (detetedRoomCenter - mapRoomCenter).norm();

            if (distance < minDistance)
            {
                minDistance = distance;
                foundMappedRoom = mapRoom;
            }
        }

        return foundMappedRoom;
    }
}