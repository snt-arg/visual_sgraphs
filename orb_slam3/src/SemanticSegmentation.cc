#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas, double segProbThreshold, int minCloudSize,
                                               std::pair<float, float> distFilterThreshold, float downsampleLeafSize)
    {
        mpAtlas = pAtlas;
        mMinCloudSize = minCloudSize;
        mSegProbThreshold = segProbThreshold;
        mDistFilterThreshold = distFilterThreshold;
        mDownsampleLeafSize = downsampleLeafSize;

        // ground planes having a height greater than this threshold over the (lowest/biggest)? ground plane are filtered
        mGroundPlaneHeightThreshold = 0.50;

        // flags to indicate what segmentation modules run, modes are:
        // 1. Semantic segmentation runs independently - mSemRuns = true, mGeoRuns = false
        // 2. Semantic segmentation runs with Geometric segmentation - mSemRuns = true, mGeoRuns = true
        // 3. Geometric segmentation runs independently - mSemRuns = false, mGeoRuns = true
        mSemRuns = true;
        mGeoRuns = true;
    }

    void SemanticSegmentation::Run()
    {
        while (true)
        {
            // Check if there are new KeyFrames in the buffer
            if (segmentedImageBuffer.empty())
            {
                usleep(3000);
                continue;
            }

            // retrieve the oldest one
            mMutexNewKFs.lock();
            std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> segImgTuple = segmentedImageBuffer.front();
            segmentedImageBuffer.pop_front();
            mMutexNewKFs.unlock();

            // separate point clouds while applying threshold
            pcl::PCLPointCloud2::Ptr pclPc2SegPrb = std::get<2>(segImgTuple);
            cv::Mat segImgUncertainity = std::get<1>(segImgTuple);
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs;
            threshSeparatePointCloud(pclPc2SegPrb, segImgUncertainity, clsCloudPtrs);

            // get the point cloud from the respective keyframe via the atlas - ignore it if KF doesn't exist
            KeyFrame *thisKF = mpAtlas->GetKeyFrameById(std::get<0>(segImgTuple));
            if (thisKF == nullptr)
                continue;
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud = thisKF->getCurrentFramePointCloud();

            // fill in class specific point clouds with XYZZ and RGB from the keyframe pointcloud
            enrichClassSpecificPointClouds(clsCloudPtrs, thisKFPointCloud);

            // clear cloud as it is no longer needed and consumes significant memory
            // [TODO] - clear even if semantic segmentation is not running
            thisKF->clearPointCloud();

            // get all planes for each class specific point cloud using RANSAC
            std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> clsPlanes =
                getPlanesFromClassClouds(clsCloudPtrs, mMinCloudSize);

            // set the class specific point clouds to the keyframe
            thisKF->setCurrentClsCloudPtrs(clsCloudPtrs);

            // Add the planes to Atlas
            updatePlaneData(thisKF, clsPlanes);

            // Check for possible room candidates
            // [TODO] Replace it with a flag
            // [TODO] Naming conflict with mGeoRuns
            bool useGeo = true; 
            if (useGeo)
                updateMapRoomCandidateToRoom_Geo();
        }
    }

    void SemanticSegmentation::AddSegmentedFrameToBuffer(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        segmentedImageBuffer.push_back(*tuple);
    }

    std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> SemanticSegmentation::GetSegmentedFrameBuffer()
    {
        return segmentedImageBuffer;
    }

    void SemanticSegmentation::threshSeparatePointCloud(pcl::PCLPointCloud2::Ptr pclPc2SegPrb,
                                                        cv::Mat &segImgUncertainity, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs)

    {
        // parse the PointCloud2 message
        const int width = pclPc2SegPrb->width;
        const int numPoints = width * pclPc2SegPrb->height;
        const int pointStep = pclPc2SegPrb->point_step;
        const int numClasses = pointStep / bytesPerClassProb;

        for (int i = 0; i < numClasses; i++)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pointCloud->is_dense = false;
            pointCloud->height = 1;
            clsCloudPtrs.push_back(pointCloud);
        }

        // apply thresholding and track confidence (complement of uncertainty)
        const uint8_t *data = pclPc2SegPrb->data.data();
        for (int j = 0; j < numClasses; j++)
        {
            for (int i = 0; i < numPoints; i++)
            {
                float value;
                memcpy(&value, data + pointStep * i + bytesPerClassProb * j + pclPc2SegPrb->fields[0].offset, bytesPerClassProb);

                if (value >= mSegProbThreshold)
                {
                    // inject coordinates as a point to respective point cloud
                    pcl::PointXYZRGBA point;
                    point.y = static_cast<int>(i / width);
                    point.x = i % width;
                    
                    // convert uncertainity to single value and assign confidence to alpha channel
                    cv::Vec3b vec = segImgUncertainity.at<cv::Vec3b>(point.y, point.x);
                    point.a = 255 - static_cast<int>(0.299 * vec[2] + 0.587 * vec[1] + 0.114 * vec[0]);

                    clsCloudPtrs[j]->push_back(point);
                }
            }
        }

        // specify size/width and log statistics
        for (int i = 0; i < numClasses; i++)
        {
            clsCloudPtrs[i]->width = clsCloudPtrs[i]->size();
            clsCloudPtrs[i]->header = pclPc2SegPrb->header;
        }
    }

    void SemanticSegmentation::enrichClassSpecificPointClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &thisKFPointCloud)
    {
        for (int i = 0; i < clsCloudPtrs.size(); i++)
        {
            for (unsigned int j = 0; j < clsCloudPtrs[i]->width; j++)
            {
                const pcl::PointXYZRGB point = thisKFPointCloud->at(clsCloudPtrs[i]->points[j].x, clsCloudPtrs[i]->points[j].y);
                clsCloudPtrs[i]->points[j].x = point.x;
                clsCloudPtrs[i]->points[j].y = point.y;
                clsCloudPtrs[i]->points[j].z = point.z;
                clsCloudPtrs[i]->points[j].r = point.r;
                clsCloudPtrs[i]->points[j].g = point.g;
                clsCloudPtrs[i]->points[j].b = point.b;
            }
            clsCloudPtrs[i]->header.frame_id = thisKFPointCloud->header.frame_id;
        }
    }

    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs, int minCloudSize)
    {
        std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> clsPlanes;

        // downsample/filter the pointcloud and extract planes
        for (unsigned long int i = 0; i < clsCloudPtrs.size(); i++)
        {
            // Downsample the given pointcloud after filtering based on distance
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud = Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(clsCloudPtrs[i], mDistFilterThreshold);
            filteredCloud = Utils::pointcloudDownsample<pcl::PointXYZRGBA>(filteredCloud, mDownsampleLeafSize);

            // copy the filtered cloud for later storage into the keyframe
            pcl::copyPointCloud(*filteredCloud, *clsCloudPtrs[i]);

            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>> extractedPlanes;
            if (filteredCloud->points.size() > minCloudSize)
            {
                extractedPlanes = Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::WeightedSACSegmentation>(filteredCloud, minCloudSize);
            }
            clsPlanes.push_back(extractedPlanes);
        }
        return clsPlanes;
    }

    void SemanticSegmentation::updatePlaneData(KeyFrame *pKF,
                                               std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> &clsPlanes)
    {
        for (size_t clsId = 0; clsId < clsPlanes.size(); clsId++)
        {
            for (auto planePoint : clsPlanes[clsId])
            {
                // Get the plane equation
                Eigen::Vector4d estimatedPlane = planePoint.second;
                g2o::Plane3D detectedPlane(estimatedPlane);

                // Convert the given plane to global coordinates
                g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                             detectedPlane);

                // Check if we need to add the wall to the map or not
                int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);

                // pointcloud processing
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = planePoint.first;
                std::vector<double> confidences;
                for (size_t i = 0; i < planeCloud->size(); i++)
                    confidences.push_back(static_cast<int>(planeCloud->points[i].a) / 255.0);
                double conf = Utils::calcSoftMin(confidences);

                // transform the planeCloud to global if semSeg uses plane->setMapClouds(planeCloud)
                if (!mGeoRuns)
                    pcl::transformPointCloud(*planeCloud, *planeCloud, pKF->GetPoseInverse().matrix().cast<float>());

                if (matchedPlaneId == -1)
                {
                    if (!mGeoRuns)
                        createMapPlane(pKF, detectedPlane, clsId, conf, planeCloud);
                }
                else
                {
                    if (!mGeoRuns)
                        updateMapPlane(pKF, detectedPlane, planeCloud, matchedPlaneId);

                    // cast a vote for the plane semantics    
                    updatePlaneSemantics(matchedPlaneId, clsId, conf);
                }

                // re-associations and filters - run them after each ground plane is detected
                if (clsId == 0)
                {
                    Plane *groundPlane = mpAtlas->GetBiggestGroundPlane();
                    if (groundPlane != nullptr)
                    {
                        // Re-compute the transformation from ground to horizontal - maybe global eq. changed
                        mPlanePoseMat = computePlaneToHorizontal(groundPlane);

                        // Filter the ground planes after the update
                        filterGroundPlanes(groundPlane);
                    }
                }
            }
        }

        // Update the ground plane, as it might have been updated
        // even when semantic segmentation did not detect any planes
        Plane *groundPlane = mpAtlas->GetBiggestGroundPlane();
        if (groundPlane != nullptr)
        {
            // Re-compute the transformation from ground to horizontal - maybe global eq. changed
            mPlanePoseMat = computePlaneToHorizontal(groundPlane);

            // Filter the ground planes
            filterGroundPlanes(groundPlane);
 
            // Re-check if all wall planes are still valid
            for (const auto &plane : mpAtlas->GetAllPlanes())
                if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL && !canBeValidWallPlane(plane))
                    plane->resetPlaneSemantics();
        }

        // reassociate semantic planes if they get close to each other :)) after optimization
        reAssociateSemanticPlanes(mpAtlas->GetAllPlanes());
    }

    void SemanticSegmentation::createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane, int clsId,
                                              double conf, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud)
    {
        ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->addObservation(pKF, estimatedPlane);

        // [TODO] - move this logic inside Atlas (or Map) for atomic IDs if multiple threads are running
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());

        // cast a vote for the plane semantics
        newMapPlane->setPlaneType(ORB_SLAM3::Plane::planeVariant::UNDEFINED);
        newMapPlane->castWeightedVote(Utils::getPlaneTypeFromClassId(clsId), conf);

        // Set the global equation of the plane
        g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                     estimatedPlane);
        newMapPlane->setGlobalEquation(globalEquation);

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

    void SemanticSegmentation::updateMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
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
        else
            for (const auto &mapPoint : pKF->GetMapPoints())
                if (Utils::pointOnPlane(currentPlane->getGlobalEquation().coeffs(), mapPoint))
                    currentPlane->setMapPoints(mapPoint);
    }

    void SemanticSegmentation::updatePlaneSemantics(int planeId, int clsId, double confidence)
    {
        // retrieve the plane from the map
        Plane *matchedPlane = mpAtlas->GetPlaneById(planeId);

        // plane type compatible with the Plane class
        ORB_SLAM3::Plane::planeVariant planeType = Utils::getPlaneTypeFromClassId(clsId);

        // cast a vote for the plane semantics
        matchedPlane->castWeightedVote(planeType, confidence);
    }

    bool SemanticSegmentation::canBeValidWallPlane(Plane *plane)
    {
        // wall validation based on the mPlanePoseMat
        // only works if the ground plane is set, needs the correction matrix: mPlanePoseMat
        Eigen::Vector3f transformedPlaneCoefficients = transformPlaneEqToGroundReference(plane->getGlobalEquation().coeffs());

        // if the transformed plane is vertical based on absolute value, then assign semantic, otherwise ignore
        // threshold should be leniently set (ideally with correct ground plane reference, this value should be close to 0.00)
        // [TODO] - Parameterize threshold
        if (abs(transformedPlaneCoefficients(1)) < 0.20)
            return true;
        return false;
    }

    void SemanticSegmentation::reAssociateSemanticPlanes(const std::vector<Plane *> &planes)
    {
        // loop through all the planes to look for associations after possible update by the optimization
        for (const auto &plane : planes)
        {
            // consider planes that have a semantic type and are not excluded from association
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED || plane->excludedFromAssoc)
                continue;

            // get plane information
            int planeId = plane->getId();

            // get the vector of all other planes with the same semantic type
            std::vector<Plane *> otherPlanes;
            for (const auto &otherPlane : planes)
                if (otherPlane->getId() != planeId && otherPlane->getPlaneType() == plane->getPlaneType())
                    otherPlanes.push_back(otherPlane);
            if (otherPlanes.empty())
                return;

            // check if the plane is associated with any other plane
            int matchedPlaneId = Utils::associatePlanes(otherPlanes, plane->getGlobalEquation());

            // if a match is found, then add the smaller planecloud to the larger plane
            // set the smaller plane type to undefined and remove it from future associations
            if (matchedPlaneId != -1)
            {
                Plane *matchedPlane = mpAtlas->GetPlaneById(matchedPlaneId);
                if (plane->getMapClouds()->points.size() < matchedPlane->getMapClouds()->points.size())
                {
                    matchedPlane->setMapClouds(plane->getMapClouds());
                    plane->resetPlaneSemantics();
                    plane->excludedFromAssoc = true;
                }
                else
                {
                    plane->setMapClouds(matchedPlane->getMapClouds());
                    matchedPlane->resetPlaneSemantics();
                    matchedPlane->excludedFromAssoc = true;
                }
            }
        }
    }

    void SemanticSegmentation::filterGroundPlanes(Plane *groundPlane)
    {
        // discard ground planes that have height above a threshold from the biggest ground plane
        // [TODO] - Whether to use biggest ground plane or lowest ground plane?

        // get the median height of the plane to compute the threshold
        float threshY = computeGroundPlaneHeight(groundPlane) - mGroundPlaneHeightThreshold;        

        // go through all ground planes to check validity
        int groundPlaneId = groundPlane->getId();
        for (const auto &plane : mpAtlas->GetAllPlanes())
        {
            if (plane->getPlaneType() != ORB_SLAM3::Plane::planeVariant::GROUND || plane->getId() == groundPlaneId)
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
            // [TODO] - Parameterize threshold
            if (abs(transformedPlaneCoefficients(0)) > 0.10)
                plane->resetPlaneSemantics();
        }
    }

    Eigen::Vector3f SemanticSegmentation::transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq)
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

    float SemanticSegmentation::computeGroundPlaneHeight(Plane *groundPlane)
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

    Eigen::Matrix4f SemanticSegmentation::computePlaneToHorizontal(const Plane *plane)
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

    void SemanticSegmentation::updateMapRoomCandidateToRoom_Geo()
    {
        // Get all the mapped planes and rooms
        std::vector<Room *> allRooms = mpAtlas->GetAllRooms();
        std::vector<Plane *> allPlanes = mpAtlas->GetAllPlanes();

        // Filter the planes to get only the walls
        std::vector<Plane *> allWalls;
        for (auto plane : allPlanes)
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
                allWalls.push_back(plane);

        // Get all the facing walls
        std::vector<std::pair<Plane *, Plane *>> facingWalls;
        for (auto wall1 : allWalls)
            for (auto wall2 : allWalls)
            {
                // Skip the same wall
                if (wall1->getId() == wall2->getId())
                    continue;

                // Check if the planes are facing each other
                bool isFacing = Utils::arePlanesFacingEachOther(wall1, wall2);
                if (isFacing)
                    facingWalls.push_back(std::make_pair(wall1, wall2));
            }

        // If there is at least one pair of facing wall
        if (facingWalls.size() > 0)
            // Loop over all the rooms
            for (auto roomCandidate : allRooms)
            {
                // If all the walls of the room candidate are detected, skip it
                int wallsNeeded = roomCandidate->getIsCorridor() ? 2 : 4;
                int wallsDetectedSoFar = roomCandidate->getWalls().size();
                if (wallsDetectedSoFar == wallsNeeded)
                    continue;

                // Get the room's candidate marker's pose
                Sophus::SE3f metaMarkerPose = roomCandidate->getMetaMarker()->getGlobalPose();

                // Find the closest facing walls to the room center
                std::pair<Plane *, Plane *> closestPair1, closestPair2;
                double minDistance1 = std::numeric_limits<double>::max();
                double minDistance2 = std::numeric_limits<double>::max();

                for (auto facingWallsPair : facingWalls)
                {
                    // Calculate distance between wall centroids and metaMarkerPose
                    double distance1 = Utils::calculateDistancePointToPlane(facingWallsPair.first->getGlobalEquation().coeffs(), metaMarkerPose.translation().cast<double>());
                    double distance2 = Utils::calculateDistancePointToPlane(facingWallsPair.second->getGlobalEquation().coeffs(), metaMarkerPose.translation().cast<double>());

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
                    { // Update the room walls
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

                // Finally, update the room candidate to a room
                roomCandidate->setIsCandidate(false);
            }
    }

    void SemanticSegmentation::updateMapRoomCandidateToRoom_Voxblox(Room *roomCandidate)
    {
        // [TODO] Needs to be implemented

        // Calculate the plane (wall) equation on which the marker is attached
        // Eigen::Vector4d planeEstimate =
        //     getPlaneEquationFromPose(currentMapMarker->getGlobalPose().rotationMatrix(),
        //                              currentMapMarker->getGlobalPose().translation());

        // // Get the plane based on the equation
        // g2o::Plane3D detectedPlane(planeEstimate);

        // // Convert the given plane to global coordinates
        // g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
        //                                                              detectedPlane);

        // // Check if we need to add the wall to the map or not
        // int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);
        // if (matchedPlaneId != -1)
        //     // The wall already exists in the map, fetching that one
        //     updateMapPlane(pKF, detectedPlane, planeCloud, matchedPlaneId, currentMapMarker);

        // Update the room center based on the meta-marker position, if it is a candidate
        // if (detectedRoom->getIsCandidate())
        //     // Update the room center
        //     detectedRoom->setRoomCenter(detectedRoom->getMetaMarker()->getGlobalPose().translation().cast<double>());

        // Find attached doors and add them to the room
        for (auto mapDoor : mpAtlas->GetAllDoors())
        {
            ORB_SLAM3::Marker *marker = mapDoor->getMarker();
            std::vector<int> roomDoorMarkerIds = roomCandidate->getDoorMarkerIds();

            // Loop over the door markers of the room
            for (auto doorMarkerId : roomDoorMarkerIds)
                if (doorMarkerId == marker->getId())
                    roomCandidate->setDoors(mapDoor);
        }

        // [TODO] Detect walls close to the room center (setWalls in SemSeg)

        // Set the new room center
        // Eigen::Vector3d updatedCentroid = Eigen::Vector3d::Zero();
        // std::vector<Plane *> roomWalls = detectedRoom->getWalls();
        // if (roomWalls.size() > 0)
        // {
        //     if (detectedRoom->getIsCorridor())
        //     {
        //         // Calculate the marker position placed on a wall
        //         Eigen::Vector3d markerPosition =
        //             roomWalls.front()->getMarkers().front()->getGlobalPose().translation().cast<double>();
        //         // If it is a corridor
        //         Eigen::Vector4d wall1(Utils::correctPlaneDirection(
        //             roomWalls.front()->getGlobalEquation().coeffs()));
        //         Eigen::Vector4d wall2(Utils::correctPlaneDirection(
        //             roomWalls.front()->getGlobalEquation().coeffs()));
        //         // Find the room center and add its vertex
        //         updatedCentroid = Utils::getRoomCenter(markerPosition, wall1, wall2);
        //     }
        //     else
        //     {
        //         reorganizeRoomWalls(detectedRoom);
        //         // If it is a four-wall room
        //         Eigen::Vector4d wall1 = Utils::correctPlaneDirection(
        //             detectedRoom->getWalls()[0]->getGlobalEquation().coeffs());
        //         Eigen::Vector4d wall2 = Utils::correctPlaneDirection(
        //             detectedRoom->getWalls()[1]->getGlobalEquation().coeffs());
        //         Eigen::Vector4d wall3 = Utils::correctPlaneDirection(
        //             detectedRoom->getWalls()[2]->getGlobalEquation().coeffs());
        //         Eigen::Vector4d wall4 = Utils::correctPlaneDirection(
        //             detectedRoom->getWalls()[3]->getGlobalEquation().coeffs());
        //         // Find the room center and add its vertex
        //         updatedCentroid = Utils::getRoomCenter(wall1, wall2, wall3, wall4);
        //     }

        //     // Update the room values
        //     detectedRoom->setRoomCenter(updatedCentroid);
        // }

        // std::cout << "- Room#" << detectedRoom->getId() << " updated (" << detectedRoom->getName()
        //           << "), augmented by Marker-ID #" << detectedRoom->getMetaMarkerId() << ", with #"
        //           << " walls and doors [" << detectedDoors << "]!"
        //           << std::endl;
    }

    void SemanticSegmentation::updateMapRoomCandidateToRoom_GNN(Room *roomCandidate)
    {
        // [TODO] Needs to be implemented
    }
}