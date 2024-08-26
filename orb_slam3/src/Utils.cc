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

    bool Utils::arePlanesFacingEachOther(const Plane *plane1, const Plane *plane2)
    {
        // Get the normal vectors of the planes
        Eigen::Vector3d normal1 = plane1->getGlobalEquation().normal();
        Eigen::Vector3d normal2 = plane2->getGlobalEquation().normal();

        // Calculate the dot product of the normals
        float dotProduct = normal1.dot(normal2);

        // Check if the dot product is close to -1
        if (dotProduct < SystemParams::GetParams()->room_seg.plane_facing_dot_thresh)
            return true;
        else
            return false;
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
        std::vector<std::pair<Plane *, Plane *>> facingPlanes;
        for (size_t idx1 = 0; idx1 < planes.size(); ++idx1)
        {
            Plane *plane1 = planes[idx1];
            for (size_t idx2 = idx1 + 1; idx2 < planes.size(); ++idx2)
            {
                Plane *plane2 = planes[idx2];
                // Check if the planes are facing each other
                bool isFacing = Utils::arePlanesFacingEachOther(plane1, plane2);
                if (isFacing)
                    facingPlanes.push_back(std::make_pair(plane1, plane2));
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

    Eigen::Vector3d Utils::getClusterCenteroid(const std::vector<Eigen::Vector3d *> &points)
    {
        // Variables
        int count = 0;
        Eigen::Vector3d sum(0.0, 0.0, 0.0);

        // Calculate the count and sum of the points
        for (const auto &pointPtr : points)
            if (pointPtr)
            {
                sum += *pointPtr;
                ++count;
            }

        // Return the centroid of the cluster
        if (count > 0)
            return sum / count;
        else
            // Handle the case where there are no points to avoid division by zero
            return Eigen::Vector3d(0.0, 0.0, 0.0);
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
                         distance = p.getVector3fMap().norm();
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

    int Utils::associatePlanes(const vector<Plane *> &mappedPlanes, g2o::Plane3D givenPlane, float threshold)
    {
        int planeId = -1;

        // Initialize difference value
        double minDiff = 100.0;

        // Check if mappedPlanes is empty
        if (mappedPlanes.empty())
            return planeId;

        // Loop over all walls
        for (const auto &mPlane : mappedPlanes)
        {
            // Skip the plane if it's excluded from association
            if (mPlane->excludedFromAssoc)
                continue;

            // Preparing a plane for feeding the detector
            g2o::Plane3D mappedPlane = mPlane->getGlobalEquation();

            // Calculate difference vector based on walls' equations
            Eigen::Vector3d diffVector = givenPlane.ominus(mappedPlane);

            // Create a single number determining the difference vector
            // [before] double planeDiff = diffVector.transpose() * Eigen::Matrix3d::Identity() * diffVector;
            double planeDiff = diffVector.norm();

            // Comparing the with minimum acceptable value
            if (planeDiff < minDiff)
            {
                minDiff = planeDiff;
                planeId = mPlane->getId();
            }
        }

        // If the difference is not large, no need to add the plane
        if (minDiff < threshold)
            return planeId;

        // Otherwise, return -1 so that the the plane gets added to the map
        return -1;
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