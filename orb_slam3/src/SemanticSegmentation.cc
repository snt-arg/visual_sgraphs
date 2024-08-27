#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas, std::vector<ORB_SLAM3::Door *> nEnvDoors,
                                               std::vector<ORB_SLAM3::Room *> nEnvRooms)
    {
        mpAtlas = pAtlas;
        envDoors = nEnvDoors;
        envRooms = nEnvRooms;

        // Get the system parameters
        sysParams = SystemParams::GetParams();

        // Set booleans according to the mode of operation
        mGeoRuns = !(sysParams->general.mode_of_operation == SystemParams::general::ModeOfOperation::SEM);
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

            // get the point cloud from the respective keyframe via the atlas - ignore it if KF doesn't exist
            KeyFrame *thisKF = mpAtlas->GetKeyFrameById(std::get<0>(segImgTuple));
            if (thisKF == nullptr)
                continue;
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud = thisKF->getCurrentFramePointCloud();
            if (thisKFPointCloud == nullptr)
            {
                std::cout << "SemSeg: skipping KF ID: " << thisKF->mnId << ". Missing pointcloud..." << std::endl;
                continue;
            }

            // separate point clouds while applying threshold
            pcl::PCLPointCloud2::Ptr pclPc2SegPrb = std::get<2>(segImgTuple);
            cv::Mat segImgUncertainity = std::get<1>(segImgTuple);
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs;
            threshSeparatePointCloud(pclPc2SegPrb, segImgUncertainity, clsCloudPtrs, thisKFPointCloud);

            // clear pointclouds as they are no longer needed and consumes significant memory
            // also clear pointclouds from the keyframes that might have been skipped
            // always keep last few keyframes as there can be minor misordering in keyframe processing
            thisKF->clearPointCloud();
            int buffer = 3;
            if (thisKF->mnId - mLastProcessedKeyFrameId > buffer)
            {
                for (unsigned long int i = mLastProcessedKeyFrameId + 1; i < thisKF->mnId - buffer; i++)
                {
                    KeyFrame *pKF = mpAtlas->GetKeyFrameById(i);
                    if (pKF != nullptr && pKF->getCurrentFramePointCloud() != nullptr)
                        pKF->clearPointCloud();
                }
                mLastProcessedKeyFrameId = thisKF->mnId - buffer;
            }

            // get all planes for each class specific point cloud using RANSAC
            std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> clsPlanes =
                getPlanesFromClassClouds(clsCloudPtrs);

            // set the class specific point clouds to the keyframe
            thisKF->setCurrentClsCloudPtrs(clsCloudPtrs);

            // Add the planes to Atlas
            updatePlaneData(thisKF, clsPlanes);

            // Check for possible room candidates
            if (sysParams->room_seg.method == SystemParams::room_seg::Method::GEOMETRIC)
                updateMapRoomCandidateToRoomGeo(thisKF);
            else if (sysParams->room_seg.method == SystemParams::room_seg::Method::FREE_SPACE)
                detectMapRoomCandidateVoxblox();
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

    void SemanticSegmentation::UpdateSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d *>> &skeletonClusterPoints)
    {
        unique_lock<std::mutex> lock(mMutexNewRooms);
        latestSkeletonCluster = skeletonClusterPoints;
    }

    std::vector<std::vector<Eigen::Vector3d *>> SemanticSegmentation::GetLatestSkeletonCluster()
    {
        return latestSkeletonCluster;
    }

    void SemanticSegmentation::threshSeparatePointCloud(pcl::PCLPointCloud2::Ptr pclPc2SegPrb, cv::Mat &segImgUncertainity,
                                                        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs,
                                                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &thisKFPointCloud)
    {
        // parse the PointCloud2 message
        const int width = pclPc2SegPrb->width;
        const int numPoints = width * pclPc2SegPrb->height;
        const int pointStep = pclPc2SegPrb->point_step;
        const int numClasses = pointStep / bytesPerClassProb;
        const float distanceThreshNear = sysParams->pointcloud.distance_thresh.first;
        const float distanceThreshFar = sysParams->pointcloud.distance_thresh.second;
        const uint8_t confidenceThresh = sysParams->sem_seg.conf_thresh * 255;
        const float probThresh = sysParams->sem_seg.prob_thresh;

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

                if (value >= probThresh)
                {
                    // inject coordinates as a point to respective point cloud
                    pcl::PointXYZRGBA point;
                    point.y = static_cast<int>(i / width);
                    point.x = i % width;

                    // get the original point from the keyframe point cloud
                    const pcl::PointXYZRGB origPoint = thisKFPointCloud->at(point.x, point.y);
                    if (!pcl::isFinite(origPoint))
                        continue;

                    // convert uncertainity to single value and assign confidence to alpha channel
                    cv::Vec3b vec = segImgUncertainity.at<cv::Vec3b>(point.y, point.x);
                    point.a = 255 - static_cast<int>(0.299 * vec[2] + 0.587 * vec[1] + 0.114 * vec[0]);

                    // exclude points with low confidence
                    if (point.a < confidenceThresh)
                        continue;

                    // assign the XYZ and RGB values to the surviving point before pushing to specific point cloud
                    point.x = origPoint.x;
                    point.y = origPoint.y;
                    point.z = origPoint.z;
                    point.r = origPoint.r;
                    point.g = origPoint.g;
                    point.b = origPoint.b;

                    // confidence as the squared inverse depth - interpolated between near and far thresholds
                    // confidence = 255 for near, 25 for far, and interpolated according to squared distance
                    const float thresholdNear = sysParams->pointcloud.distance_thresh.first;
                    const float thresholdFar = sysParams->pointcloud.distance_thresh.second;
                    if (point.z < thresholdNear)
                        point.a = 255;
                    else if (point.z > thresholdFar)
                        point.a = 25;
                    else
                        point.a = 255 - static_cast<int>(230 * sqrt((point.z - thresholdNear) / (thresholdFar - thresholdNear)));

                    // add the point to the respective class specific point cloud
                    clsCloudPtrs[j]->push_back(point);
                }
            }
        }

        // specify size/width and header for each class specific point cloud
        for (int i = 0; i < numClasses; i++)
        {
            clsCloudPtrs[i]->width = clsCloudPtrs[i]->size();
            clsCloudPtrs[i]->header = pclPc2SegPrb->header;
        }
    }

    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs)
    {
        std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> clsPlanes;

        // downsample/filter the pointcloud and extract planes
        for (size_t i = 0; i < clsCloudPtrs.size(); i++)
        {
            // [TODO?] - Perhaps consider points in order of confidence instead of downsampling
            // Downsample the given pointcloud after filtering based on distance
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;
            filteredCloud = Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(clsCloudPtrs[i]);
            filteredCloud = Utils::pointcloudDownsample<pcl::PointXYZRGBA>(filteredCloud,
                                                                           sysParams->sem_seg.pointcloud.downsample.leaf_size,
                                                                           sysParams->sem_seg.pointcloud.downsample.min_points_per_voxel);
            filteredCloud = Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(filteredCloud,
                                                                               sysParams->sem_seg.pointcloud.outlier_removal.std_threshold,
                                                                               sysParams->sem_seg.pointcloud.outlier_removal.mean_threshold);

            // copy the filtered cloud for later storage into the keyframe
            pcl::copyPointCloud(*filteredCloud, *clsCloudPtrs[i]);

            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>> extractedPlanes;
            if (filteredCloud->points.size() > sysParams->seg.pointclouds_thresh)
            {
                extractedPlanes = Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::WeightedSACSegmentation>(filteredCloud);
            }
            clsPlanes.push_back(extractedPlanes);
        }
        return clsPlanes;
    }

    void SemanticSegmentation::updatePlaneData(KeyFrame *pKF,
                                               std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                                                                 Eigen::Vector4d>>> &clsPlanes)
    {
        for (size_t clsId = 0; clsId < clsPlanes.size(); clsId++)
        {
            for (auto planePoint : clsPlanes[clsId])
            {
                // Get the plane equation
                Eigen::Vector4d estimatedPlane = planePoint.second;
                g2o::Plane3D detectedPlane(estimatedPlane);

                // Convert the given plane to global coordinates
                g2o::Plane3D globalEquation = Utils::applyPoseToPlane(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                      detectedPlane);

                // Check if we need to add the wall to the map or not
                int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation, sysParams->seg.plane_association_thresh);

                // pointcloud processing - compute the average confidence across all pixels in the plane observation
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = planePoint.first;
                std::vector<double> confidences;
                for (size_t i = 0; i < planeCloud->size(); i++)
                    confidences.push_back(static_cast<int>(planeCloud->points[i].a) / 255.0);
                double conf = Utils::calcSoftMin(confidences);

                if (matchedPlaneId == -1)
                {
                    if (!mGeoRuns)
                    {
                        ORB_SLAM3::Plane *newMapPlane = GeoSemHelpers::createMapPlane(mpAtlas, pKF, detectedPlane, planeCloud, conf);
                        // Cast a vote for the plane semantics
                        updatePlaneSemantics(newMapPlane->getId(), clsId, conf);
                    }
                }
                else
                {
                    if (!mGeoRuns)
                        GeoSemHelpers::updateMapPlane(mpAtlas, pKF, detectedPlane, planeCloud, matchedPlaneId, conf);
                    else
                    {
                        pcl::transformPointCloud(*planeCloud, *planeCloud, pKF->GetPoseInverse().matrix().cast<float>());
                        ORB_SLAM3::Plane *matchedPlane = mpAtlas->GetPlaneById(matchedPlaneId);
                        // Add the plane cloud to the matched plane
                        if (!planeCloud->empty())
                            matchedPlane->setMapClouds(planeCloud);
                    }

                    // Cast a vote for the plane semantics
                    updatePlaneSemantics(matchedPlaneId, clsId, conf);
                }

                // re-associations and filters - run them after each ground plane is detected
                if (clsId == 0)
                {
                    Plane *mainGroundPlane = mpAtlas->GetBiggestGroundPlane();
                    if (mainGroundPlane != nullptr)
                    {
                        // Re-compute the transformation from ground to horizontal - maybe global eq. changed
                        mPlanePoseMat = computePlaneToHorizontal(mainGroundPlane);

                        // Filter the ground planes after the update
                        filterGroundPlanes(mainGroundPlane);
                    }
                }
            }
        }

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

        // reassociate semantic planes if they get close to each other :)) after optimization
        if (sysParams->sem_seg.reassociate.enabled)
            reAssociateSemanticPlanes();
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

    void SemanticSegmentation::filterWallPlanes()
    {
        for (const auto &plane : mpAtlas->GetAllPlanes())
        {
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
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

    void SemanticSegmentation::reAssociateSemanticPlanes()
    {
        // loop through all the planes to look for associations after possible update by the optimization
        const std::vector<Plane *> planes = mpAtlas->GetAllPlanes();
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
            int matchedPlaneId = Utils::associatePlanes(otherPlanes, plane->getGlobalEquation(), sysParams->sem_seg.reassociate.association_thresh);

            // if a match is found, then add the smaller planecloud to the larger plane
            // set the smaller plane type to undefined and remove it from future associations
            if (matchedPlaneId != -1)
            {
                Plane *matchedPlane = mpAtlas->GetPlaneById(matchedPlaneId);
                Plane *smallPlane, *bigPlane;
                if (plane->getMapClouds()->points.size() < matchedPlane->getMapClouds()->points.size())
                {
                    smallPlane = plane;
                    bigPlane = matchedPlane;
                }
                else
                {
                    smallPlane = matchedPlane;
                    bigPlane = plane;
                }

                // add the smaller planecloud to the bigger plane
                bigPlane->setMapClouds(smallPlane->getMapClouds());

                // add all map points of the smaller plane to the bigger plane
                for (const auto &mapPoint : smallPlane->getMapPoints())
                    bigPlane->setMapPoints(mapPoint);

                // push all observations of the smaller plane to the bigger plane
                for (const auto &obs : smallPlane->getObservations())
                    bigPlane->addObservation(obs.first, obs.second);

                // reset the smaller plane semantics
                smallPlane->resetPlaneSemantics();
                smallPlane->excludedFromAssoc = true;

                std::cout << "Plane " << smallPlane->getId() << " merged with Plane " << bigPlane->getId() << std::endl;
            }
        }
    }

    void SemanticSegmentation::filterGroundPlanes(Plane *groundPlane)
    {
        // discard ground planes that have height above a threshold from the biggest ground plane
        // [TODO] - Whether to use biggest ground plane or lowest ground plane?

        // get the median height of the plane to compute the threshold
        float threshY = computeGroundPlaneHeight(groundPlane) - sysParams->sem_seg.max_step_elevation;

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
            if (abs(transformedPlaneCoefficients(0)) > sysParams->sem_seg.max_tilt_ground)
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

    bool SemanticSegmentation::getRectangularRoom(
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
    void SemanticSegmentation::updateMapRoomCandidateToRoomGeo(KeyFrame *pKF)
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

                // Finally, update the room candidate to a room
                roomCandidate->setIsCandidate(false);
            }
    }

    void SemanticSegmentation::detectMapRoomCandidateVoxblox()
    {
        // Variables
        std::vector<Plane *> allWalls, closestWalls;
        ORB_SLAM3::Room *newClusterBasedRoom = nullptr;

        // Get the skeleton clusters
        std::vector<std::vector<Eigen::Vector3d *>> clusters = GetLatestSkeletonCluster();

        // Get all the mapped planes and rooms
        std::vector<Plane *> allPlanes = mpAtlas->GetAllPlanes();

        // Filter the planes to get only the walls
        for (auto plane : allPlanes)
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
                allWalls.push_back(plane);

        // Find the walls closest to the cluster points
        for (auto cluster : clusters)
        {
            // Initializations
            closestWalls.clear();
            std::pair<std::pair<Plane *, Plane *>, std::pair<Plane *, Plane *>> rectangularRoom;

            // Loop over all walls
            for (const auto &wall : allWalls)
            {
                // Loop over all cluster points
                for (const auto &point : cluster)
                {
                    // Calculate distance between wall centroid and cluster centroid
                    double distance = Utils::calculateDistancePointToPlane(wall->getGlobalEquation().coeffs(), *point);
                    // If the distance is smaller than the threshold, add the wall to closestWalls
                    if (distance < sysParams->room_seg.cluster_point_wall_distance_thresh)
                    {
                        // Add the wall to closestWalls
                        closestWalls.push_back(wall);
                        break;
                    }
                }
            }

            // Get all the facing walls
            std::vector<std::pair<Plane *, Plane *>> facingWalls =
                Utils::getAllPlanesFacingEachOther(closestWalls);

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
                    // If the room already exists, update the existing room candidate with the new information
                    std::cout
                        << "- New room detected: Room#" << newClusterBasedRoom->getId()
                        << " using the free-space!" << std::endl;
                    mpAtlas->AddDetectedMapRoom(newClusterBasedRoom);
                }
            }
        }
    }

    void SemanticSegmentation::detectMapRoomCandidateGNN()
    {
        // [TODO] Needs to be implemented
    }

    ORB_SLAM3::Room *SemanticSegmentation::roomAssociation(const ORB_SLAM3::Room *givenRoom,
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