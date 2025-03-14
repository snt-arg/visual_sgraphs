#include "Utils.h"

namespace ORB_SLAM3
{
    double Utils::calculateEuclideanDistance(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2)
    {
        double dx = p1.x() - p2.x();
        double dy = p1.y() - p2.y();
        double dz = p1.z() - p2.z();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double Utils::calculateDistancePointToPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &point)
    {
        // Find the distance of the point from a given plane
        return fabs(plane.head<3>().dot(point) + plane(3));
    }

    Eigen::Vector3d Utils::lineIntersectsPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &lineStart, const Eigen::Vector3d &lineEnd)
    {
        // Calculate the direction vector of the line
        Eigen::Vector3d lineDirection = lineEnd - lineStart;

        // [TODO] - check if the line is parallel to the plane

        // Calculate the intersection point
        double t = -(plane.head<3>().dot(lineStart) + plane(3)) / plane.head<3>().dot(lineDirection);
        return lineStart + t * lineDirection;
    }

    bool Utils::arePlanesFacingEachOther(const Plane *plane1, const Plane *plane2)
    {
        // Get the normal vectors of the planes
        Eigen::Vector3d normal1 = plane1->getGlobalEquation().normal();
        Eigen::Vector3d normal2 = plane2->getGlobalEquation().normal();

        // Calculate the dot product of the normals
        double dotProduct = normal1.dot(normal2);

        // Check if the dot product is close to -1, indicating opposite directions
        return dotProduct < SystemParams::GetParams()->room_seg.plane_facing_dot_thresh;
    }

    bool Utils::arePlanesApartEnough(const Plane *plane1, const Plane *plane2, const double &threshold)
    {
        // Correct the directions of both planes to ensure consistent orientation
        Eigen::Vector4d v1 = correctPlaneDirection(plane1->getGlobalEquation().coeffs());
        Eigen::Vector4d v2 = correctPlaneDirection(plane2->getGlobalEquation().coeffs());

        // Extract normals after correction
        Eigen::Vector3d normal1 = v1.head<3>();
        Eigen::Vector3d normal2 = v2.head<3>();

        // Check if planes are parallel by evaluating the dot product of their normals
        double dotProduct = normal1.dot(normal2) / (normal1.norm() * normal2.norm());
        if (std::abs(dotProduct) > 0.99) // Threshold close to 1 for near-parallel normals
        {
            // Calculate the perpendicular distance between the planes
            double distance = std::abs(v1(3) - v2(3)) / normal1.norm();

            // Print distance and threshold
            // std::cout << "- Distance between walls #" << plane1->getId() << " and #" << plane2->getId() << ": "
            //           << distance << std::endl;

            // Check if the calculated distance is greater than the threshold
            return distance > threshold;
        }
        else
        {
            // Planes are not parallel (they intersect), so return false
            std::cout << "- Planes #" << plane1->getId() << " and #" << plane2->getId() << " are not parallel." << std::endl;
            return false;
        }
    }

    bool Utils::arePlanesPerpendicular(const Plane *plane1, const Plane *plane2, const double &threshold)
    {
        // Calculate the dot product of the normal vectors of the planes
        Eigen::Vector3d normal1 = plane1->getGlobalEquation().normal();
        Eigen::Vector3d normal2 = plane2->getGlobalEquation().normal();
        double dotProduct = normal1.dot(normal2);

        // Calculate the angle between the planes
        double angle = std::acos(std::abs(dotProduct));

        // Check if the angle is within the threshold
        return std::abs(angle - M_PI / 2.0) < threshold;
    }

    std::vector<std::pair<Plane *, Plane *>> Utils::getAllPlanesFacingEachOther(const std::vector<Plane *> &planes)
    {
        // Variables
        SystemParams *sysParams = SystemParams::GetParams();
        std::vector<std::pair<Plane *, Plane *>> facingPlanes;
        double minValidSpace = sysParams->room_seg.min_wall_distance_thresh;

        // Loop through all the planes
        for (size_t idx1 = 0; idx1 < planes.size(); ++idx1)
        {
            Plane *plane1 = planes[idx1];
            for (size_t idx2 = idx1 + 1; idx2 < planes.size(); ++idx2)
            {
                // Variables
                Plane *plane2 = planes[idx2];
                // Check if the planes are facing each other
                bool isFacing = Utils::arePlanesFacingEachOther(plane1, plane2);
                if (isFacing)
                {
                    if (Utils::arePlanesApartEnough(plane1, plane2, minValidSpace))
                        facingPlanes.push_back(std::make_pair(plane1, plane2));
                }
            }
        }
        return facingPlanes;
    }

    Eigen::Vector4d Utils::correctPlaneDirection(const Eigen::Vector4d &plane)
    {
        // Check if the transformation is needed
        if (plane(3) > 0)
            return -plane;
        else
            return plane;
    }

    g2o::Plane3D Utils::applyPoseToPlane(const Eigen::Matrix4d &kfPose, const g2o::Plane3D &plane)
    {
        Eigen::Vector4d v = plane.coeffs();
        Eigen::Vector4d v2;
        Eigen::Matrix3d R = kfPose.block<3, 3>(0, 0);
        v2.head<3>() = R * v.head<3>();
        v2(3) = v(3) - kfPose.block<3, 1>(0, 3).dot(v2.head<3>());
        return g2o::Plane3D(v2);
    }

    Eigen::Vector3d Utils::getClusterCenteroid(const std::vector<Eigen::Vector3d> &points)
    {
        // Check if there are points in the vector
        if (points.empty())
            return Eigen::Vector3d(0.0, 0.0, 0.0);

        // Variables
        Eigen::Vector3d sum(0.0, 0.0, 0.0);

        // Calculate the sum of the points
        for (const auto &point : points)
            sum += point;

        // Return the centroid of the cluster
        return sum / points.size();
    }

    Eigen::Vector3d Utils::getRoomCenter(const Eigen::Vector3d &givenPoint,
                                         const Eigen::Vector4d &wall1,
                                         const Eigen::Vector4d &wall2)
    {
        Eigen::Vector3d roomCenter;
        Eigen::Vector3d vec, vectorNormal;

        // Get the dominant wall by comparing the magnitudes of the last elements of the given walls
        if (fabs(wall1(3)) > fabs(wall2(3)))
            // Calculate the midpoint of the dominant wall
            vec = (0.5 * (fabs(wall1(3)) * wall1.head(3) - fabs(wall2(3)) * wall2.head(3))) +
                  fabs(wall2(3)) * wall2.head(3);
        else
            // Calculate the midpoint of the dominant wall
            vec = (0.5 * (fabs(wall2(3)) * wall2.head(3) - fabs(wall1(3)) * wall1.head(3))) +
                  fabs(wall1(3)) * wall1.head(3);

        // Normalize the vector to obtain the normal direction of the room
        vectorNormal = vec / vec.norm();

        // Calculate the room center by projecting the marker position onto the room plane
        roomCenter = vec + (givenPoint - (givenPoint.dot(vectorNormal)) * vectorNormal);

        return roomCenter;
    }

    Eigen::Vector3d Utils::getRoomCenter(const Eigen::Vector4d x_plane1, const Eigen::Vector4d x_plane2,
                                         const Eigen::Vector4d y_plane1, const Eigen::Vector4d y_plane2)
    {
        Eigen::Vector3d roomCenter;
        Eigen::Vector3d vectorX, vectorY;

        // Calculate the midpoint vector along the x-axis of the room
        if (fabs(x_plane1(3)) > fabs(x_plane2(3)))
            vectorX = (0.5 * (fabs(x_plane1(3)) * x_plane1.head(3) - fabs(x_plane2(3)) * x_plane2.head(3))) +
                      fabs(x_plane2(3)) * x_plane2.head(3);
        else
            vectorX = (0.5 * (fabs(x_plane2(3)) * x_plane2.head(3) - fabs(x_plane1(3)) * x_plane1.head(3))) +
                      fabs(x_plane1(3)) * x_plane1.head(3);

        // Calculate the midpoint vector along the y-axis of the room
        if (fabs(y_plane1(3)) > fabs(y_plane2(3)))
            vectorY = (0.5 * (fabs(y_plane1(3)) * y_plane1.head(3) - fabs(y_plane2(3)) * y_plane2.head(3))) +
                      fabs(y_plane2(3)) * y_plane2.head(3);
        else
            vectorY = (0.5 * (fabs(y_plane2(3)) * y_plane2.head(3) - fabs(y_plane1(3)) * y_plane1.head(3))) +
                      fabs(y_plane1(3)) * y_plane1.head(3);

        // Calculate the room center by summing the midpoint vectors along the x and y axes
        roomCenter = vectorX + vectorY;

        return roomCenter;
    }

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudDownsample(
        const typename pcl::PointCloud<PointT>::Ptr &cloud, const float leafSize, const unsigned int minPointsPerVoxel)
    {
        // The filtered point cloud object
        typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());

        // Define the downsampling filter
        typename pcl::VoxelGrid<PointT>::Ptr downsampleFilter(new pcl::VoxelGrid<PointT>());

        // Set the parameters of the downsampling filter
        downsampleFilter->setLeafSize(leafSize, leafSize, leafSize);
        downsampleFilter->setMinimumPointsNumberPerVoxel(minPointsPerVoxel);
        downsampleFilter->setInputCloud(cloud);

        // Apply the downsampling filter
        downsampleFilter->filter(*filteredCloud);
        filteredCloud->header = cloud->header;

        return filteredCloud;
    }
    template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &, const float, const unsigned int);

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudDistanceFilter(
        const typename pcl::PointCloud<PointT>::Ptr &cloud)
    {
        // Variables
        double distance;
        const std::pair<float, float> thresholds = SystemParams::GetParams()->pointcloud.distance_thresh;
        const float thresholdNear = thresholds.first;
        const float thresholdFar = thresholds.second;

        // Define the filtered point cloud object
        typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
        filteredCloud->reserve(cloud->size());

        // Filter the point cloud
        std::copy_if(cloud->begin(),
                     cloud->end(),
                     std::back_inserter(filteredCloud->points),
                     [&](const PointT &p)
                     {
                         //  distance = p.getVector3fMap().norm();
                         distance = p.z;
                         return distance > thresholdNear && distance < thresholdFar;
                     });

        filteredCloud->height = 1;
        filteredCloud->is_dense = false;
        filteredCloud->header = cloud->header;
        filteredCloud->width = filteredCloud->size();

        return filteredCloud;
    }
    template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudOutlierRemoval(
        const typename pcl::PointCloud<PointT>::Ptr &cloud, const int meanThresh, const float stdDevThresh)
    {
        // Check if the input cloud is empty
        if (cloud->points.size() == 0)
            return cloud;

        // Create a container for the filtered cloud
        typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);

        // Create the filtering object: StatisticalOutlierRemoval
        pcl::StatisticalOutlierRemoval<PointT> outlierRemoval;
        outlierRemoval.setInputCloud(cloud);
        outlierRemoval.setMeanK(meanThresh);
        outlierRemoval.setStddevMulThresh(stdDevThresh);
        outlierRemoval.filter(*filteredCloud);

        // Return the filtered cloud
        return filteredCloud;
    }
    template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &, const int, const float);

    template <typename PointT, template <typename> class SegmentationType>
    std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>> Utils::ransacPlaneFitting(
        typename pcl::PointCloud<PointT>::Ptr &cloud)
    {
        std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>> extractedPlanes;
        SystemParams *sysParams = SystemParams::GetParams();

        for (unsigned int i = 0; i < sysParams->seg.ransac.max_planes && cloud->points.size() > sysParams->seg.pointclouds_thresh; i++)
        {
            try
            {
                // Create objects for RANSAC plane segmentation
                typename pcl::ExtractIndices<PointT> extract;
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

                // Create the SAC segmentation object
                SegmentationType<PointT> seg;

                // Fill the values of the segmentation object
                seg.setInputCloud(cloud);
                seg.setNumberOfThreads(8);
                seg.setMaxIterations(sysParams->seg.ransac.max_iterations);
                seg.setDistanceThreshold(sysParams->seg.ransac.distance_thresh);
                seg.setOptimizeCoefficients(true);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setModelType(pcl::SACMODEL_PLANE);

                // Apply RANSAC segmentation
                seg.segment(*inliers, *coeffs);

                // Calculate normal on the plane
                Eigen::Vector4d planeEquation(coeffs->values[0], coeffs->values[1],
                                              coeffs->values[2], coeffs->values[3]);

                // Calculate the closest points
                Eigen::Vector4d plane;
                Eigen::Vector3d closestPoint = planeEquation.head(3) * planeEquation(3);
                plane.head(3) = closestPoint / closestPoint.norm();
                plane(3) = closestPoint.norm();

                // Create a new point cloud containing the points of the detected planes
                typename pcl::PointCloud<PointT>::Ptr extractedCloud(new pcl::PointCloud<PointT>);
                for (const auto &idx : inliers->indices)
                {
                    // Fill the point cloud
                    PointT inPoint;
                    inPoint.r = cloud->points[idx].r;
                    inPoint.g = cloud->points[idx].g;
                    inPoint.b = cloud->points[idx].b;
                    inPoint.x = cloud->points[idx].x;
                    inPoint.y = cloud->points[idx].y;
                    inPoint.z = cloud->points[idx].z;
                    inPoint.a = cloud->points[idx].a;

                    // Add the point to the cloud
                    extractedCloud->points.push_back(inPoint);
                }

                // Add the extracted cloud to the vector
                extractedPlanes.push_back(std::make_pair(extractedCloud, plane));

                // Extract the inliers
                extract.setInputCloud(cloud);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*cloud);
            }
            catch (const std::exception &e)
            {
                std::cout << "RANSAC model error!" << std::endl;
            }
        }
        return extractedPlanes;
    }
    template std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::SACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
    template std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::WeightedSACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

    ORB_SLAM3::Plane::planeVariant Utils::getPlaneTypeFromClassId(int clsId)
    {
        switch (clsId)
        {
        case 0:
            return ORB_SLAM3::Plane::planeVariant::GROUND;
        case 1:
            return ORB_SLAM3::Plane::planeVariant::WALL;
        default:
            return ORB_SLAM3::Plane::planeVariant::UNDEFINED;
        }
    }

    int Utils::getClassIdFromPlaneType(ORB_SLAM3::Plane::planeVariant planeType)
    {
        switch (planeType)
        {
        case ORB_SLAM3::Plane::planeVariant::GROUND:
            return 0;
        case ORB_SLAM3::Plane::planeVariant::WALL:
            return 1;
        default:
            return -1;
        }
    }

    bool Utils::pointOnPlane(Eigen::Vector4d planeEquation, MapPoint *mapPoint)
    {
        if (mapPoint->isBad())
            return false;

        // Find the distance of the point from a given plane
        double pointPlaneDist = calculateDistancePointToPlane(planeEquation, mapPoint->GetWorldPos().cast<double>());

        // Apply a threshold
        if (pointPlaneDist < SystemParams::GetParams()->seg.plane_point_dist_thresh)
            return true;

        return false;
    }

    int Utils::associatePlanes(const vector<Plane *> &mappedPlanes,
                               g2o::Plane3D givenPlane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr givenCloud,
                               const Eigen::Matrix4d &kfPose,
                               const Plane::planeVariant obsPlaneType,
                               const float threshold)
    {
        int planeId = -1;

        // Initialize difference value
        double minDiff = 100.0;

        // Candidate planes for clustering check
        std::vector<std::pair<Plane *, double>> checksForCluster;

        // Check if mappedPlanes is empty
        if (mappedPlanes.empty())
            return planeId;

        // Get the centroid of the given plane
        if (givenCloud->empty())
            return planeId;
        Eigen::Vector3d givenCentroid;
        givenCentroid.setZero();
        for (const auto &point : *givenCloud)
        {
            Eigen::Vector3d pointVec(point.x, point.y, point.z);
            givenCentroid += pointVec;
        }
        givenCentroid /= givenCloud->size();

        SystemParams *sysParams = SystemParams::GetParams();

        // Loop over all planes
        for (const auto &mPlane : mappedPlanes)
        {
            // Skip the plane if it's excluded from association
            if (mPlane->excludedFromAssoc || mPlane->getMapClouds()->empty())
                continue;

            if ((obsPlaneType != Plane::planeVariant::UNDEFINED) && (mPlane->getExpectedPlaneType() != obsPlaneType))
                continue;

            // convert mapped plane to local frame of given plane
            g2o::Plane3D mappedPlane = Utils::applyPoseToPlane(kfPose, mPlane->getGlobalEquation());

            // Calculate difference vector based on plane equations
            // given plane is assumed to be in the frame represented by kfPose
            Eigen::Vector3d diffVector = givenPlane.ominus(mappedPlane);

            // a maximum distance threshold - last element of the diffVector
            if (diffVector(2) > sysParams->seg.plane_association.distance_thresh)
                continue;

            // Create a single number determining the difference vector
            double planeDiff = diffVector.norm();

            // check ominus threshold
            if (planeDiff > threshold)
                continue;

            // check if centroid is far enough to check for clustering
            Eigen::Vector3d mappedCentroid = mPlane->getCentroid().cast<double>();
            double centroidDiff = (givenCentroid - mappedCentroid).norm();
            if (sysParams->seg.plane_association.cluster_separation.enabled
                // && obsPlaneType == Plane::planeVariant::WALL // perform clustering only for walls
                && centroidDiff > sysParams->seg.plane_association.centroid_thresh)
            {
                checksForCluster.push_back(std::make_pair(mPlane, planeDiff));
                continue;
            }

            // Comparing the with minimum value
            if (planeDiff < minDiff)
            {
                minDiff = planeDiff;
                planeId = mPlane->getId();
            }
        }

        // check for clustering only if no plane is found - reduces unnecessary expensive clustering
        if (sysParams->seg.plane_association.cluster_separation.enabled && planeId == -1 && !checksForCluster.empty())
        {
            for (const auto &check : checksForCluster)
            {
                Plane *mPlane = check.first;
                double planeDiff = check.second;

                // check if it's a different cluster
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::copyPointCloud(*mPlane->getMapClouds(), *aggregatedCloud);
                if (givenCloud->size() < 1000)
                    aggregatedCloud = pointcloudDownsample<pcl::PointXYZRGBA>(aggregatedCloud,
                                                                              sysParams->seg.plane_association.cluster_separation.downsample.leaf_size,
                                                                              sysParams->seg.plane_association.cluster_separation.downsample.min_points_per_voxel);
                for (const auto &point : *givenCloud)
                {
                    pcl::PointXYZRGBA newPoint;
                    pcl::copyPoint(point, newPoint);
                    aggregatedCloud->push_back(newPoint);
                }
                if (givenCloud->size() >= 1000)
                    aggregatedCloud = pointcloudDownsample<pcl::PointXYZRGBA>(aggregatedCloud,
                                                                              sysParams->seg.plane_association.cluster_separation.downsample.leaf_size,
                                                                              sysParams->seg.plane_association.cluster_separation.downsample.min_points_per_voxel);
                if (aggregatedCloud->empty())
                    continue;

                std::vector<pcl::PointIndices> clusterIndices;
                clusterPlaneClouds(aggregatedCloud, clusterIndices);
                if (clusterIndices.size() > 1)
                    continue;

                // check the difference
                if (planeDiff < minDiff)
                {
                    minDiff = planeDiff;
                    planeId = mPlane->getId();
                }
            }
        }

        return planeId;
    }

    void Utils::clusterPlaneClouds(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, std::vector<pcl::PointIndices> &clusterIndices)
    {
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance(SystemParams::GetParams()->seg.plane_association.cluster_separation.tolerance);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(2500000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);
    }

    void Utils::reAssociateSemanticPlanes(Atlas *mpAtlas)
    {
        // Variables
        SystemParams *sysParams = SystemParams::GetParams();
        const std::vector<Plane *> planes = mpAtlas->GetAllPlanes();

        for (const auto &plane : planes)
        {
            // Only consider planes with a semantic type and not excluded from association
            if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED || plane->excludedFromAssoc)
                continue;

            // Get plane information
            int planeId = plane->getId();

            // Get the vector of all other planes with the same semantic type
            std::vector<Plane *> otherPlanes;
            for (const auto &otherPlane : planes)
                if (otherPlane->getId() != planeId && otherPlane->getPlaneType() == plane->getPlaneType())
                    otherPlanes.push_back(otherPlane);

            // Skip if there are no other planes with the same semantic type
            if (otherPlanes.empty())
                return;

            // Check if the plane is associated with any other plane
            int matchedPlaneId = associatePlanes(otherPlanes,
                                                 plane->getGlobalEquation(),
                                                 plane->getMapClouds(),
                                                 Eigen::Matrix4d::Identity(),
                                                 plane->getPlaneType(),
                                                 sysParams->sem_seg.reassociate.association_thresh);

            // If a match is found, then add the smaller planecloud to the larger plane
            // set the smaller plane type to undefined and remove it from future associations
            if (matchedPlaneId != -1)
            {
                // Variables
                Plane *smallPlane, *bigPlane;
                Plane *matchedPlane = mpAtlas->GetPlaneById(matchedPlaneId);

                // if one of them has a plane type, then it is automatically the bigger plane
                if (matchedPlane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::UNDEFINED ||
                    plane->getMapClouds()->points.size() > matchedPlane->getMapClouds()->points.size())
                {
                    smallPlane = matchedPlane;
                    bigPlane = plane;
                }
                else
                {
                    smallPlane = plane;
                    bigPlane = matchedPlane;
                }

                // Add the smaller planecloud to the bigger plane
                bigPlane->setMapClouds(smallPlane->getMapClouds());

                // Add all map points of the smaller plane to the bigger plane
                for (const auto &mapPoint : smallPlane->getMapPoints())
                    bigPlane->setMapPoints(mapPoint);

                // Push all observations of the smaller plane to the bigger plane
                for (const auto &obs : smallPlane->getObservations())
                    bigPlane->addObservation(obs.first, obs.second);

                // Reset the smaller plane semantics
                smallPlane->resetPlaneSemantics();
                smallPlane->excludedFromAssoc = true;

                std::cout << "- Re-associating planes #" << smallPlane->getId() << " & #"
                          << bigPlane->getId() << " ..." << std::endl;
            }
        }
    }

    double Utils::calcSoftMin(vector<double> &values)
    {
        // parameter controlling the softness/sharpness of the soft-min
        // the smaller the value, the more conservative the soft-min
        const double tau = 0.1;

        // soft-min = sum(exp(-value/tau) * value) / sum(exp(-value/tau))
        Eigen::Map<Eigen::VectorXd> confs(values.data(), values.size());
        Eigen::VectorXd term = ((1.0 - confs.array()) / tau).exp();
        return ((term / term.sum()).array() * confs.array()).sum();
    }
}