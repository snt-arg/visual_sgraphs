#include "GeometricSegmentation.h"

namespace ORB_SLAM3
{
    GeometricSegmentation::GeometricSegmentation(Atlas *pAtlas, bool hasDepthCloud)
    {
        mpAtlas = pAtlas;
        mHasDepthCloud = hasDepthCloud;

        // Get the system parameters
        sysParams = SystemParams::GetParams();
    }

    void GeometricSegmentation::AddKeyFrameToBuffer(KeyFrame *pKF)
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        mvpKeyFrameBuffer.push_back(pKF);
    }

    std::list<KeyFrame *> GeometricSegmentation::GetKeyFrameBuffer()
    {
        return mvpKeyFrameBuffer;
    }

    void GeometricSegmentation::Run()
    {
        while (!(sysParams->general.mode_of_operation == SystemParams::general::ModeOfOperation::SEM))
        {
            // Check if there are new KeyFrames in the buffer
            if (mvpKeyFrameBuffer.empty())
            {
                usleep(3000);
                continue;
            }

            // Get the first KeyFrame in the buffer
            mMutexNewKFs.lock();
            KeyFrame *mpCurrentKeyFrame = mvpKeyFrameBuffer.front();
            mvpKeyFrameBuffer.pop_front();
            mMutexNewKFs.unlock();

            // Fetch the planes from the current KeyFrame
            fetchPlanesFromKeyFrame(mpCurrentKeyFrame, mHasDepthCloud);

            // clear the current KeyFrame pointcloud if semantic mode is not active
            if (sysParams->general.mode_of_operation == SystemParams::general::ModeOfOperation::GEO)
                mpCurrentKeyFrame->clearPointCloud();

            usleep(3000);
        }
    }

    void GeometricSegmentation::fetchPlanesFromKeyFrame(ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud)
    {
        // Get the plane equation from the points the camera is seeing
        std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>> planePointVec;
        planePointVec = getPlanesFromPointClouds(pKF, hasDepthCloud);

        // Loop through all the planes detected
        for (auto planePoint : planePointVec)
        {
            // Get the plane equation
            Eigen::Vector4d planeEstimate = planePoint.second;
            g2o::Plane3D detectedPlane(planeEstimate);
            
            // Convert the given plane to global coordinates
            g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                         detectedPlane);

            // convert planeCloud to global coordinates
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = planePoint.first;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::transformPointCloud(*planeCloud, *globalPlaneCloud, pKF->GetPoseInverse().matrix().cast<float>());

            // Check if we need to add the wall to the map or not
            int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);
            if (matchedPlaneId == -1)
                // A wall with the same equation was not found in the map, creating a new one
                createMapPlane(pKF, detectedPlane, globalPlaneCloud);
            else
                // The wall already exists in the map, fetching that one
                updateMapPlane(pKF, detectedPlane, globalPlaneCloud, matchedPlaneId);

            // Add Markers while progressing in KFs
            markerSemanticDetectionAndMapping(pKF, pKF->getCurrentFrameMarkers(), planeCloud);
        }
    }

    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>> GeometricSegmentation::getPlanesFromPointClouds(
        ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud)
    {
        // Variables
        std::vector<g2o::Plane3D> detectedPlanes;
        std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>> extractedPlanes;

        // Based on the depth information, calculate the plane equation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;

        if (hasDepthCloud)
            pointcloud = pKF->getCurrentFramePointCloud();
        else
            // [TODO] For Mono and Stereo, the map points are very sparse. We can think of a way to calculate
            // the depth from points using Machine Learning to get a better plane estimate.
            pointcloud = getCloudFromSparsePoints(pKF->getCurrentFrameMapPoints()); // mCurrentFrame.mvpMapPoints

        // Convert the pointcloud to one with PointXYZRGBA for consistency
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*pointcloud, *cloudRGBA);
        
        // Downsample the given pointcloud after filtering based on distance
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud = Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(cloudRGBA);
        filteredCloud = Utils::pointcloudDownsample<pcl::PointXYZRGBA>(filteredCloud, sysParams->geo_seg.downsample_leaf_size);

        if (filteredCloud->points.size() > sysParams->seg.pointclouds_thresh)
        {
            extractedPlanes = Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::SACSegmentation>(filteredCloud);
        }
        return extractedPlanes;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeometricSegmentation::getCloudFromSparsePoints(const std::vector<MapPoint *> &points)
    {
        // Create a point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Add all points to the cloud
        for (const auto mapPoint : points)
        {
            pcl::PointXYZRGB pcl_point;
            pcl_point.x = mapPoint->GetWorldPos()(0);
            pcl_point.y = mapPoint->GetWorldPos()(1);
            pcl_point.z = mapPoint->GetWorldPos()(2);
            cloud->points.push_back(pcl_point);
        }
        // Return the cloud
        return cloud;
    }

    void GeometricSegmentation::createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                               const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud)
    {
        ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->addObservation(pKF, estimatedPlane);
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());

        // Set the plane type to undefined, as it is not known yet
        newMapPlane->setPlaneType(ORB_SLAM3::Plane::planeVariant::UNDEFINED);

        // Set the global equation of the plane
        g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                     estimatedPlane);
        newMapPlane->setGlobalEquation(globalEquation);

        // std::string planeStr = "(" +
        //                        std::to_string(newMapPlane->getGlobalEquation().coeffs()(0)) + ")x + (" +
        //                        std::to_string(newMapPlane->getGlobalEquation().coeffs()(1)) + ")y + (" +
        //                        std::to_string(newMapPlane->getGlobalEquation().coeffs()(2)) + ")z + (" +
        //                        std::to_string(newMapPlane->getGlobalEquation().coeffs()(3)) + ") = 0";
        // std::cout << "- New plane detected: Plane#" << newMapPlane->getId() << ", Eq: "
        //           << planeStr << std::endl;

        // Fill the plane with the pointcloud
        if (!planeCloud->points.empty())
            newMapPlane->setMapClouds(planeCloud);
        else
            // Loop to find the points lying on wall
            for (const auto &mapPoint : mpAtlas->GetAllMapPoints())
                if (Utils::pointOnPlane(newMapPlane->getGlobalEquation().coeffs(), mapPoint))
                    newMapPlane->setMapPoints(mapPoint);

        pKF->AddMapPlane(newMapPlane);
        mpAtlas->AddMapPlane(newMapPlane);
    }

    void GeometricSegmentation::updateMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                                               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                                               int planeId, ORB_SLAM3::Marker *visitedMarker)
    {
        // Find the matched plane among all planes of the map
        Plane *currentPlane = mpAtlas->GetPlaneById(planeId);
        currentPlane->addObservation(pKF, estimatedPlane);

        // Add the plane to the list of planes in the current KeyFrame
        pKF->AddMapPlane(currentPlane);

        // If there is a marker attached to a plane, set it as 'wall' and store the marker
        // if (visitedMarker != NULL)
        // {
        //     currentPlane->setMarkers(visitedMarker);
        //     std::cout << "- Wall found: Plane#" << currentPlane->getId() << ", with Marker#"
        //               << visitedMarker->getId() << std::endl;
        // }

        // Update the pointcloud of the plane
        if (!planeCloud->points.empty())
            currentPlane->setMapClouds(planeCloud);
        else
            for (const auto &mapPoint : pKF->GetMapPoints())
                if (Utils::pointOnPlane(currentPlane->getGlobalEquation().coeffs(), mapPoint))
                    currentPlane->setMapPoints(mapPoint);
    }

    Eigen::Vector4d GeometricSegmentation::getPlaneEquationFromPose(const Eigen::Matrix3f &rotationMatrix,
                                                                    const Eigen::Vector3f &translation)
    {
        // Get the normal of the plane from the rotation matrix
        Eigen::Vector3f normal = rotationMatrix.col(2); // Assuming the Z-axis represents the normal direction

        // Get a point on the plane from the translation
        Eigen::Vector3f pointOnPlane = -translation;

        // Calculate the D coefficient of the plane equation
        double D = normal.dot(pointOnPlane);

        // Return the plane equation [A, B, C, D]
        return Eigen::Vector4d(normal.x(), normal.y(), normal.z(), D);
    }

    ORB_SLAM3::Marker *GeometricSegmentation::createMapMarker(const ORB_SLAM3::Marker *visitedMarker,
                                                              ORB_SLAM3::KeyFrame *pKF)
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

    void GeometricSegmentation::createMapDoor(ORB_SLAM3::Marker *attachedMarker, ORB_SLAM3::KeyFrame *pKF, std::string name)
    {
        // Check if the door has not been created before
        bool doorAlreadyInMap = false;
        for (auto door : mpAtlas->GetAllDoors())
            if (door->getMarker()->getId() == attachedMarker->getId())
                doorAlreadyInMap = true;

        if (doorAlreadyInMap)
            return;

        ORB_SLAM3::Door *newMapDoor = new ORB_SLAM3::Door();

        newMapDoor->setName(name);
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

    void GeometricSegmentation::createMapRoomCandidate(ORB_SLAM3::Room *matchedRoom, ORB_SLAM3::Marker *attachedMarker)
    {
        // Check if the room has not been created before
        bool roomAlreadyInMap = false;
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
            << "- New room candidate detected: Room#" << newMapRoomCandidate->getId() << " (" << newMapRoomCandidate->getName()
            << "), augmented by Marker-ID #" << newMapRoomCandidate->getMetaMarkerId() << "!" << std::endl;

        mpAtlas->AddMapRoom(newMapRoomCandidate);
    }

    void GeometricSegmentation::markerSemanticDetectionAndMapping(ORB_SLAM3::KeyFrame *pKF,
                                                                  const std::vector<Marker *> &mvpMapMarkers,
                                                                  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud)
    {
        for (Marker *mCurrentMarker : mvpMapMarkers)
        {
            // Variables
            ORB_SLAM3::Marker *currentMapMarker;

            // Check the type of the marker
            std::pair<bool, std::string> result = Utils::isMarkerAttachedToDoor(mCurrentMarker->getId(), envDoors);
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
                currentMapMarker = createMapMarker(mCurrentMarker, pKF);
            }
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
                createMapDoor(mCurrentMarker, pKF, doorName);
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
                        createMapRoomCandidate(envRoom, mCurrentMarker);
            }
        }
    }

    ORB_SLAM3::Room *GeometricSegmentation::roomAssociation(const ORB_SLAM3::Room *givenRoom)
    {
        // Variables
        double minDistance = 100;
        ORB_SLAM3::Room *foundMappedRoom = nullptr;

        // Get the given room center
        Eigen::Vector3d detetedRoomCenter = givenRoom->getRoomCenter();

        // Check to find the room with the minimum distance from the center
        for (const auto &mapRoom : mpAtlas->GetAllRooms())
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