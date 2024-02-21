#include "GeometricSegmentation.h"

namespace ORB_SLAM3
{
    GeometricSegmentation::GeometricSegmentation(Atlas *pAtlas, bool hasDepthCloud, int minCloudSize,
                                                 std::pair<float, float> distFilterThreshold, float downsampleLeafSize)
    {
        mpAtlas = pAtlas;
        mMinCloudSize = minCloudSize;
        mHasDepthCloud = hasDepthCloud;
        mDistFilterThreshold = distFilterThreshold;
        mDownsampleLeafSize = downsampleLeafSize;
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
        while (1)
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
            fetchPlanesFromKeyFrame(mpCurrentKeyFrame, mHasDepthCloud, mMinCloudSize);

            usleep(3000);
        }
    }

    void GeometricSegmentation::fetchPlanesFromKeyFrame(ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud, int minCloudSize)
    {
        // Variables
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> planePointVec;

        // Get the plane equation from the points the camera is seeing
        planePointVec = getPlanesFromPointClouds(pKF, hasDepthCloud, minCloudSize);

        // Loop through all the planes detected
        for (auto planePoint : planePointVec)
        {
            // Get the plane equation from the points
            Eigen::Vector4d planeEstimate(planePoint->back().normal_x, planePoint->back().normal_y,
                                          planePoint->back().normal_z, planePoint->back().curvature);
            g2o::Plane3D detectedPlane(planeEstimate);
            // Convert the given plane to global coordinates
            g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                         detectedPlane);

            // Check if we need to add the wall to the map or not
            int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);
            if (matchedPlaneId == -1)
                // A wall with the same equation was not found in the map, creating a new one
                createMapPlane(pKF, detectedPlane, planePoint);
            else
                // The wall already exists in the map, fetching that one
                updateMapPlane(pKF, detectedPlane, planePoint, matchedPlaneId);

            // Add Markers while progressing in KFs
            markerSemanticDetectionAndMapping(pKF, pKF->getCurrentFrameMarkers(), planePoint);
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> GeometricSegmentation::getPlanesFromPointClouds(
        ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud, int minCloudSize)
    {
        // Variables
        std::vector<g2o::Plane3D> detectedPlanes;
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> extractedPlanes;

        // Based on the depth information, calculate the plane equation
        std::vector<Eigen::Vector4d> planeEstimatesFromPoints;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;

        if (hasDepthCloud)
            pointcloud = pKF->getCurrentFramePointCloud();
        else
            // [TODO] For Mono and Stereo, the map points are very sparse. We can think of a way to calculate
            // the depth from points using Machine Learning to get a better plane estimate.
            pointcloud = getCloudFromSparsePoints(pKF->getCurrentFrameMapPoints()); // mCurrentFrame.mvpMapPoints

        // [TODO] decide when to downsample and/or distance filter

        // // Downsample the given pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud = Utils::pointcloudDownsample<pcl::PointXYZRGB>(pointcloud, mDownsampleLeafSize);

        // Filter the pointcloud based on a range of distance
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud = Utils::pointcloudDistanceFilter(downsampledCloud, mDistFilterThreshold);

        if (filteredCloud->points.size() > minCloudSize)
        {
            //  Estimate the plane equation
            extractedPlanes = Utils::ransacPlaneFitting(filteredCloud, minCloudSize);
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
                                               const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud)
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

        std::string planeStr = "(" +
                               std::to_string(newMapPlane->getGlobalEquation().coeffs()(0)) + ")x + (" +
                               std::to_string(newMapPlane->getGlobalEquation().coeffs()(1)) + ")y + (" +
                               std::to_string(newMapPlane->getGlobalEquation().coeffs()(2)) + ")z + (" +
                               std::to_string(newMapPlane->getGlobalEquation().coeffs()(3)) + ") = 0";
        std::cout << "- New plane detected: Plane#" << newMapPlane->getId() << ", Eq: "
                  << planeStr << std::endl;

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
                                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud,
                                               int planeId, ORB_SLAM3::Marker *visitedMarker)
    {
        // Find the matched plane among all planes of the map
        Plane *currentPlane = mpAtlas->GetPlaneById(planeId);

        // If there is a marker attached to a plane, set it as 'wall' and store the marker
        if (visitedMarker != NULL)
        {
            currentPlane->setMarkers(visitedMarker);
            currentPlane->addObservation(pKF, estimatedPlane);
            // std::cout << "- Wall found: Plane#" << currentPlane->getId() << ", with Marker#"
            //           << visitedMarker->getId() << std::endl;
        }

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
        newMapMarker->setOpId(visitedMarker->getOpId());
        newMapMarker->setId(visitedMarker->getId());
        newMapMarker->setTime(visitedMarker->getTime());
        newMapMarker->setMarkerInGMap(visitedMarker->isMarkerInGMap());
        newMapMarker->setLocalPose(visitedMarker->getLocalPose());
        newMapMarker->SetMap(mpAtlas->GetCurrentMap());
        newMapMarker->setGlobalPose(visitedMarker->getGlobalPose());
        newMapMarker->setMarkerType(visitedMarker->getMarkerType());
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
        {
            if (door->getMarker()->getId() == attachedMarker->getId())
            {
                doorAlreadyInMap = true;
            }
        }

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

    void GeometricSegmentation::markerSemanticDetectionAndMapping(ORB_SLAM3::KeyFrame *pKF,
                                                                  const std::vector<Marker *> &mvpMapMarkers,
                                                                  const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud)
    {
        for (Marker *mCurrentMarker : mvpMapMarkers)
        {
            ORB_SLAM3::Marker *currentMapMarker;
            if (!mCurrentMarker->isMarkerInGMap())
            {
                mCurrentMarker->SetMap(mpAtlas->GetCurrentMap());
                mCurrentMarker->setGlobalPose(pKF->GetPoseInverse() * mCurrentMarker->getLocalPose());
                mCurrentMarker->setMarkerInGMap(true);

                // Creating a new marker in the map
                currentMapMarker = createMapMarker(mCurrentMarker, pKF);
            }
            else
            {
                for (auto currentMapMarkerAtlas : mpAtlas->GetAllMarkers())
                {
                    if (currentMapMarkerAtlas->getId() == mCurrentMarker->getId())
                    {
                        currentMapMarker = currentMapMarkerAtlas;
                        currentMapMarker->addObservation(pKF, mCurrentMarker->getLocalPose());
                    }
                }
            }

            // Check the current marker if it is attached to a door or a wall
            std::pair<bool, std::string> result = Utils::isMarkerAttachedToDoor(currentMapMarker->getId(), envDoors);
            bool markerIsDoor = result.first;
            if (!markerIsDoor)
            {
                // Calculate the plane (wall) equation on which the marker is attached
                Eigen::Vector4d planeEstimate =
                    getPlaneEquationFromPose(currentMapMarker->getGlobalPose().rotationMatrix(),
                                             currentMapMarker->getGlobalPose().translation());

                // Get the plane based on the equation
                g2o::Plane3D detectedPlane(planeEstimate);

                // Convert the given plane to global coordinates
                g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                             detectedPlane);

                // Check if we need to add the wall to the map or not
                int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);
                if (matchedPlaneId != -1)
                    // The wall already exists in the map, fetching that one
                    updateMapPlane(pKF, detectedPlane, planeCloud, matchedPlaneId, currentMapMarker);
            }
            else
            {
                // The current marker is placed on a door
                std::string doorName = result.second;
                createMapDoor(mCurrentMarker, pKF, doorName);
            }
        }
    }
}