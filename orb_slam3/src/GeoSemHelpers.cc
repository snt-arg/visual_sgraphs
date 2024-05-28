#include "GeoSemHelpers.h"

namespace ORB_SLAM3
{
    ORB_SLAM3::Plane *GeoSemHelpers::createMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                                    const g2o::Plane3D estimatedPlane,
                                                    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud)
    {
        ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->addObservation(pKF, estimatedPlane);
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());
        newMapPlane->referenceKeyFrame = pKF;

        // Set the plane type to undefined, as it is not known yet
        newMapPlane->setPlaneType(ORB_SLAM3::Plane::planeVariant::UNDEFINED);

        // Set the global equation of the plane
        g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                     estimatedPlane);
        newMapPlane->setGlobalEquation(globalEquation);

        // Fill the plane with the pointcloud
        if (!planeCloud->points.empty())
            newMapPlane->setMapClouds(planeCloud);

        // Loop to find the points lying on wall
        // for (const auto &mapPoint : mpAtlas->GetAllMapPoints())
        //     if (Utils::pointOnPlane(newMapPlane->getGlobalEquation().coeffs(), mapPoint))
        //         newMapPlane->setMapPoints(mapPoint);

        pKF->AddMapPlane(newMapPlane);
        mpAtlas->AddMapPlane(newMapPlane);

        return newMapPlane;
    }

    void GeoSemHelpers::updateMapPlane(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId)
    {
        // Find the matched plane among all planes of the map
        Plane *currentPlane = mpAtlas->GetPlaneById(planeId);
        currentPlane->addObservation(pKF, estimatedPlane);

        // Add the plane to the list of planes in the current KeyFrame
        pKF->AddMapPlane(currentPlane);

        // Update the pointcloud of the plane
        if (!planeCloud->points.empty())
            currentPlane->setMapClouds(planeCloud);

        // for (const auto &mapPoint : pKF->GetMapPoints())
        //     if (Utils::pointOnPlane(currentPlane->getGlobalEquation().coeffs(), mapPoint))
        //         currentPlane->setMapPoints(mapPoint);
    }

    std::pair<bool, std::string> GeoSemHelpers::checkIfMarkerIsDoor(const int &markerId,
                                                                    std::vector<ORB_SLAM3::Door *> envDoors)
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

    void GeoSemHelpers::markerSemanticAnalysis(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                               std::vector<ORB_SLAM3::Door *> envDoors,
                                               std::vector<ORB_SLAM3::Room *> envRooms)
    {
        // Get the markers from the current KeyFrame
        std::vector<Marker *> mvpMapMarkers = pKF->getCurrentFrameMarkers();

        for (Marker *mCurrentMarker : mvpMapMarkers)
        {
            // Variables
            ORB_SLAM3::Marker *currentMapMarker;

            // Check the type of the marker
            std::pair<bool, std::string> result = checkIfMarkerIsDoor(mCurrentMarker->getId(), envDoors);
            bool markerIsDoor = result.first;
            std::string doorName = result.second;

            // Change the marker type
            mCurrentMarker->setMarkerType(markerIsDoor
                                              ? ORB_SLAM3::Marker::markerVariant::ON_DOOR
                                              : ORB_SLAM3::Marker::markerVariant::ON_ROOM_CENTER);

            // If the marker is not in the map, add it
            if (!mCurrentMarker->isMarkerInGMap())
            {
                mCurrentMarker->SetMap(mpAtlas->GetCurrentMap());
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
        newMapMarker->SetMap(mpAtlas->GetCurrentMap());
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

    void GeoSemHelpers::createMapDoor(Atlas *mpAtlas, ORB_SLAM3::KeyFrame *pKF,
                                      ORB_SLAM3::Marker *attachedMarker, std::string doorName)
    {
        // Check if the door has not been created before
        bool doorAlreadyInMap = false;
        for (auto door : mpAtlas->GetAllDoors())
            if (door->getMarker()->getId() == attachedMarker->getId())
                doorAlreadyInMap = true;

        if (doorAlreadyInMap)
            return;

        // Variables
        ORB_SLAM3::Door *newMapDoor = new ORB_SLAM3::Door();

        newMapDoor->setName(doorName);
        newMapDoor->setMarker(attachedMarker);
        newMapDoor->SetMap(mpAtlas->GetCurrentMap());
        newMapDoor->setId(mpAtlas->GetAllDoors().size());
        newMapDoor->setLocalPose(attachedMarker->getLocalPose());
        newMapDoor->setGlobalPose(attachedMarker->getGlobalPose());

        std::cout << "- New door detected: Door#" << newMapDoor->getId() << " (" << newMapDoor->getName()
                  << "), with Marker#" << attachedMarker->getId() << std::endl;

        pKF->AddMapDoor(newMapDoor);
        mpAtlas->AddMapDoor(newMapDoor);
    }

    void GeoSemHelpers::organizeRoomWalls(ORB_SLAM3::Room *givenRoom)
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

    void GeoSemHelpers::createMapRoomCandidateByMarker(Atlas *mpAtlas, ORB_SLAM3::Room *matchedRoom,
                                                       ORB_SLAM3::Marker *attachedMarker)
    {
        // Variables
        bool roomAlreadyInMap = false;

        // Check if a room has not been created before for this marker
        for (auto room : mpAtlas->GetAllRooms())
            if (room->getMetaMarkerId() == attachedMarker->getId())
                roomAlreadyInMap = true;

        if (roomAlreadyInMap)
            return;

        // Variables
        ORB_SLAM3::Room *newMapRoomCandidate = new ORB_SLAM3::Room();

        // Fill the room entity
        newMapRoomCandidate->setIsCandidate(true);
        newMapRoomCandidate->setMetaMarker(attachedMarker);
        newMapRoomCandidate->setName(matchedRoom->getName());
        newMapRoomCandidate->SetMap(mpAtlas->GetCurrentMap());
        newMapRoomCandidate->setId(mpAtlas->GetAllRooms().size());
        newMapRoomCandidate->setIsCorridor(matchedRoom->getIsCorridor());
        newMapRoomCandidate->setMetaMarkerId(matchedRoom->getMetaMarkerId());
        newMapRoomCandidate->setRoomCenter(attachedMarker->getGlobalPose().translation().cast<double>());

        for (int markerId : matchedRoom->getDoorMarkerIds())
            newMapRoomCandidate->setDoorMarkerIds(markerId);

        std::cout
            << "- New room candidate detected: Room#" << newMapRoomCandidate->getId() << " ("
            << newMapRoomCandidate->getName() << ") using marker #" << attachedMarker->getId() << "!\n";

        mpAtlas->AddMapRoom(newMapRoomCandidate);
    }

    void GeoSemHelpers::createMapRoomCandidateByFreeSpace(Atlas *mpAtlas, bool isCorridor,
                                                          Eigen::Vector3d clusterCentroid,
                                                          std::vector<ORB_SLAM3::Plane *> walls)
    {
        // Variables
        bool roomAlreadyInMap = false;

        // [TODO] Check if a room has not been created before for this marker

        if (roomAlreadyInMap)
            return;

        // Variables
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        ORB_SLAM3::Room *newMapRoomCandidate = new ORB_SLAM3::Room();

        // Fill the room entity
        newMapRoomCandidate->setIsCandidate(true);
        newMapRoomCandidate->setIsCorridor(isCorridor);
        newMapRoomCandidate->SetMap(mpAtlas->GetCurrentMap());
        newMapRoomCandidate->setId(mpAtlas->GetAllRooms().size());
        newMapRoomCandidate->setName(isCorridor ? "Unknown Corridor" : "Unknown Room");

        // Connect the walls to the room
        for (ORB_SLAM3::Plane *wall : walls)
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

        std::cout
            << "- New room candidate detected: Room#" << newMapRoomCandidate->getId()
            << " using the free-space!" << std::endl;

        mpAtlas->AddMapRoom(newMapRoomCandidate);
    }
}