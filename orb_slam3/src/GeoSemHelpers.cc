#include "GeoSemHelpers.h"

namespace VS_GRAPHS
{
    VS_GRAPHS::Plane *GeoSemHelpers::createMapPlane(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF,
                                                    const g2o::Plane3D estimatedPlane,
                                                    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                                                    VS_GRAPHS::Plane::planeVariant semanticType, double confidence)
    {
        VS_GRAPHS::Plane *newMapPlane = new VS_GRAPHS::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());
        newMapPlane->refKeyFrame = pKF;

        // The observation of the plane
        VS_GRAPHS::Plane::Observation obs;

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
        newMapPlane->setPlaneType(VS_GRAPHS::Plane::planeVariant::UNDEFINED);

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

    void GeoSemHelpers::updateMapPlane(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId,
                                       VS_GRAPHS::Plane::planeVariant semanticType, double confidence)
    {
        // Find the matched plane among all planes of the map
        Plane *currentPlane = mpAtlas->GetPlaneById(planeId);

        // The observation of the plane
        VS_GRAPHS::Plane::Observation obs;

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

    std::pair<bool, std::string> GeoSemHelpers::checkIfMarkerIsDoor(const int &markerId,
                                                                    std::vector<VS_GRAPHS::Door *> envDoors)
    {
        bool isDoor = false;
        std::string name = "";
        // Loop over all markers attached to doors
        for (const auto &doorPtr : envDoors)
        {
            if (doorPtr->getMarkerId() == markerId)
            {
                isDoor = true;
                name = doorPtr->getName();
                break; // No need to continue searching if found
            }
        }
        // Returning
        return std::make_pair(isDoor, name);
    }

    void GeoSemHelpers::markerSemanticAnalysis(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF,
                                               std::vector<VS_GRAPHS::Door *> envDoors,
                                               std::vector<VS_GRAPHS::Room *> envRooms)
    {
        // Get the markers from the current KeyFrame
        std::vector<Marker *> mvpMapMarkers = pKF->getCurrentFrameMarkers();

        for (Marker *mCurrentMarker : mvpMapMarkers)
        {
            // Variables
            VS_GRAPHS::Marker *currentMapMarker;

            // Check the type of the marker
            std::pair<bool, std::string> result = checkIfMarkerIsDoor(mCurrentMarker->getId(), envDoors);
            bool markerIsDoor = result.first;
            std::string doorName = result.second;

            // Change the marker type
            mCurrentMarker->setMarkerType(markerIsDoor
                                              ? VS_GRAPHS::Marker::markerVariant::ON_DOOR
                                              : VS_GRAPHS::Marker::markerVariant::ON_ROOM_CENTER);

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
            if (markerIsDoor)
                // The current marker is placed on a door
                createMapDoor(mpAtlas, pKF, mCurrentMarker, doorName);
            else
            {
                // The current marker is a room meta-marker
                VS_GRAPHS::Room *mappedRoom;

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

    VS_GRAPHS::Marker *GeoSemHelpers::createMapMarker(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF,
                                                      const VS_GRAPHS::Marker *visitedMarker)
    {
        VS_GRAPHS::Marker *newMapMarker = new VS_GRAPHS::Marker();

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

    void GeoSemHelpers::createMapDoor(Atlas *mpAtlas, VS_GRAPHS::KeyFrame *pKF,
                                      VS_GRAPHS::Marker *attachedMarker, std::string doorName)
    {
        // Check if the door has not been created before
        bool doorAlreadyInMap = false;
        for (auto door : mpAtlas->GetAllDoors())
            if (door->getMarker()->getId() == attachedMarker->getId())
                doorAlreadyInMap = true;

        if (doorAlreadyInMap)
            return;

        // Variables
        VS_GRAPHS::Door *newMapDoor = new VS_GRAPHS::Door();

        newMapDoor->setName(doorName);
        newMapDoor->setMarker(attachedMarker);
        newMapDoor->setMap(mpAtlas->GetCurrentMap());
        newMapDoor->setId(mpAtlas->GetAllDoors().size());
        newMapDoor->setLocalPose(attachedMarker->getLocalPose());
        newMapDoor->setGlobalPose(attachedMarker->getGlobalPose());

        std::cout << "- New door detected: Door#" << newMapDoor->getId() << " (" << newMapDoor->getName()
                  << "), with Marker#" << attachedMarker->getId() << std::endl;

        pKF->AddMapDoor(newMapDoor);
        mpAtlas->AddMapDoor(newMapDoor);
    }

    void GeoSemHelpers::organizeRoomWalls(VS_GRAPHS::Room *givenRoom)
    {
        // Function to find the minimum and maximum coefficient value of a wall among all walls
        auto findExtremumWall = [&](const std::vector<Plane *> &walls, int coeffIndex, bool findMax) -> Plane *
        {
            Plane *extremumWall = walls.front();
            for (const auto &wall : walls)
            {
                if ((findMax && wall->getGlobalEquation().coeffs()(coeffIndex) > extremumWall->getGlobalEquation().coeffs()(coeffIndex)) ||
                    (!findMax && wall->getGlobalEquation().coeffs()(coeffIndex) < extremumWall->getGlobalEquation().coeffs()(coeffIndex)))
                {
                    extremumWall = wall;
                }
            }
            return extremumWall;
        };

        // Get all walls in the room
        const std::vector<Plane *> &walls = givenRoom->getWalls();

        // Find walls with minimum and maximum coefficients along x and z axes
        Plane *maxWallX = findExtremumWall(walls, 0, true);
        Plane *maxWallZ = findExtremumWall(walls, 2, true);
        Plane *minWallX = findExtremumWall(walls, 0, false);
        Plane *minWallZ = findExtremumWall(walls, 2, false);

        givenRoom->clearWalls();
        givenRoom->setWalls(minWallX);
        givenRoom->setWalls(maxWallX);
        givenRoom->setWalls(minWallZ);
        givenRoom->setWalls(maxWallZ);
    }

    void GeoSemHelpers::createMapRoomCandidateByMarker(Atlas *mpAtlas, VS_GRAPHS::Room *matchedRoom,
                                                       VS_GRAPHS::Marker *attachedMarker)
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
        VS_GRAPHS::Room *newMapRoomCandidate = new VS_GRAPHS::Room();

        // Fill the room entity
        newMapRoomCandidate->setHasKnownLabel(true);
        newMapRoomCandidate->setMetaMarker(attachedMarker);
        newMapRoomCandidate->setName(matchedRoom->getName());
        newMapRoomCandidate->setMap(mpAtlas->GetCurrentMap());
        newMapRoomCandidate->setId(mpAtlas->GetAllRooms().size());
        newMapRoomCandidate->setIsCorridor(matchedRoom->getIsCorridor());
        newMapRoomCandidate->setMetaMarkerId(matchedRoom->getMetaMarkerId());
        newMapRoomCandidate->setRoomCenter(attachedMarker->getGlobalPose().translation().cast<double>());

        // Add door markers to the room
        for (int markerId : matchedRoom->getDoorMarkerIds())
        {
            newMapRoomCandidate->setDoorMarkerIds(markerId);
            // Check if the door marker is already in the map
            for (auto door : mpAtlas->GetAllDoors())
                if (door->getMarker()->getId() == markerId)
                    newMapRoomCandidate->setDoors(door);
        }

        std::cout
            << "- New room candidate detected: Room#" << newMapRoomCandidate->getId() << " ("
            << newMapRoomCandidate->getName() << ") using marker #" << attachedMarker->getId() << "!\n";

        mpAtlas->AddMarkerBasedMapRoom(newMapRoomCandidate);
    }

    VS_GRAPHS::Room *GeoSemHelpers::createMapRoomCandidateByFreeSpace(Atlas *mpAtlas, bool isCorridor,
                                                                      std::vector<VS_GRAPHS::Plane *> walls,
                                                                      Eigen::Vector3d clusterCentroid)
    {
        // Variables
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        VS_GRAPHS::Room *newMapRoomCandidate = new VS_GRAPHS::Room();

        // Fill the room entity
        newMapRoomCandidate->setHasKnownLabel(false);
        newMapRoomCandidate->setIsCorridor(isCorridor);
        newMapRoomCandidate->setMap(mpAtlas->GetCurrentMap());
        newMapRoomCandidate->setId(mpAtlas->GetAllRooms().size());

        // Set name based on ID
        int roomId = newMapRoomCandidate->getId();
        std::string roomType = isCorridor ? "Corridor" : "Room";
        newMapRoomCandidate->setName(roomType + "#" + std::to_string(roomId));

        // Connect the walls to the room
        for (VS_GRAPHS::Plane *wall : walls)
            newMapRoomCandidate->setWalls(wall);

        // Detect the room center
        if (isCorridor)
        {
            // Refining the walls of the corridor
            Eigen::Vector4d wall1(Utils::correctPlaneDirection(
                walls[0]->getGlobalEquation().coeffs()));
            Eigen::Vector4d wall2(Utils::correctPlaneDirection(
                walls[1]->getGlobalEquation().coeffs()));
            // Find the room center and add its vertex
            centroid = Utils::getRoomCenter(clusterCentroid, wall1, wall2);
        }
        else
        {
            // Reorganize the walls of the room
            organizeRoomWalls(newMapRoomCandidate);
            // Compute the centroid of the walls
            Eigen::Vector4d wall1 = Utils::correctPlaneDirection(
                walls[0]->getGlobalEquation().coeffs());
            Eigen::Vector4d wall2 = Utils::correctPlaneDirection(
                walls[1]->getGlobalEquation().coeffs());
            Eigen::Vector4d wall3 = Utils::correctPlaneDirection(
                walls[2]->getGlobalEquation().coeffs());
            Eigen::Vector4d wall4 = Utils::correctPlaneDirection(
                walls[3]->getGlobalEquation().coeffs());
            // Find the room center and add its vertex
            centroid = Utils::getRoomCenter(wall1, wall2, wall3, wall4);
        }
        newMapRoomCandidate->setRoomCenter(centroid);

        return newMapRoomCandidate;
    }

    void GeoSemHelpers::augmentMapRoomCandidate(VS_GRAPHS::Room *markerBasedRoom, VS_GRAPHS::Room *clusterBasedRoom,
                                                bool isMarkerBasedMapped)
    {
        std::cout << "\n[GeoSeg]" << std::endl;
        if (isMarkerBasedMapped)
        {
            // Augment the already detected marker-based room with the cluster-based room information
            markerBasedRoom->setRoomCenter(clusterBasedRoom->getRoomCenter());
            // Connect the walls to the room
            for (VS_GRAPHS::Plane *wall : clusterBasedRoom->getWalls())
                markerBasedRoom->setWalls(wall);
            // Create a room candidate for it
            std::cout << "- Marker-based room candidate #" << markerBasedRoom->getId()
                      << " has been validated (walls added)!" << std::endl;
        }
        else
        {
            // Augment the already detected cluster-based room with the marker-based room information
            clusterBasedRoom->setHasKnownLabel(true);
            clusterBasedRoom->setName(markerBasedRoom->getName());
            clusterBasedRoom->setIsCorridor(markerBasedRoom->getIsCorridor());
            clusterBasedRoom->setMetaMarker(markerBasedRoom->getMetaMarker());
            clusterBasedRoom->setMetaMarkerId(markerBasedRoom->getMetaMarkerId());
            // Connect the doors to the room
            for (VS_GRAPHS::Door *door : markerBasedRoom->getDoors())
                clusterBasedRoom->setDoors(door);
            for (int markerId : markerBasedRoom->getDoorMarkerIds())
                clusterBasedRoom->setDoorMarkerIds(markerId);
            // Create a room candidate for it
            std::cout << "- Cluster-based room candidate #" << clusterBasedRoom->getId()
                      << " has been validated (semantic info added)!" << std::endl;
        }
    }

    void GeoSemHelpers::associateGroundPlaneToRoom(Atlas *mpAtlas, VS_GRAPHS::Room *givenRoom)
    {
        std::vector<VS_GRAPHS::Plane *> allWalls = givenRoom->getWalls();
        VS_GRAPHS::Plane *associatedGroundPlane = nullptr;
        size_t maxInliers = 0;

        // get the ground planes from the Atlas
        std::vector<VS_GRAPHS::Plane *> groundPlanes;
        for (const auto &plane : mpAtlas->GetAllPlanes())
            if (plane->getPlaneType() == VS_GRAPHS::Plane::planeVariant::GROUND)
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

    size_t GeoSemHelpers::countGroundPlanePointsWithinWalls(std::vector<VS_GRAPHS::Plane *> &roomWalls, VS_GRAPHS::Plane *groundPlane)
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

}