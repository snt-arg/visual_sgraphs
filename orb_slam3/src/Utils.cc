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
        // Get a point on the plane
        Eigen::Vector3d pointOnPlane = plane.head<3>() * plane(3);
        // Calculate the distance |(P - A) . N|
        return fabs((point - pointOnPlane).dot(plane.head<3>()));
    }

    bool Utils::arePlanesFacingEachOther(const Plane *plane1, const Plane *plane2)
    {
        // Variables
        float threshold = -0.9f;

        // Get the normal vectors of the planes
        Eigen::Vector3d normal1 = plane1->getGlobalEquation().normal();
        Eigen::Vector3d normal2 = plane2->getGlobalEquation().normal();

        // Calculate the dot product of the normals
        float dotProduct = normal1.dot(normal2);

        // Check if the dot product is close to -1
        if (dotProduct < threshold)
            return true;
        else
            return false;
    }

    Eigen::Vector4d Utils::correctPlaneDirection(const Eigen::Vector4d &plane)
    {
        // Check if the transformation is needed
        if (plane(3) > 0)
            return -plane;
        else
            return plane;
    }

    g2o::Plane3D Utils::convertToGlobalEquation(const Eigen::Matrix4d &kfPose, const g2o::Plane3D &plane)
    {
        Eigen::Vector4d v = plane.coeffs();
        Eigen::Vector4d v2;
        Eigen::Matrix3d R = kfPose.block<3, 3>(0, 0);
        v2.head<3>() = R * v.head<3>();
        v2(3) = v(3) - kfPose.block<3, 1>(0, 3).dot(v2.head<3>());
        return g2o::Plane3D(v2);
    };

    Eigen::Vector3d Utils::getRoomCenter(const Eigen::Vector3d &markerPosition,
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
        roomCenter = vec + (markerPosition - (markerPosition.dot(vectorNormal)) * vectorNormal);

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

    std::pair<bool, std::string> Utils::isMarkerAttachedToDoor(const int &markerId,
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

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudDownsample(
        const typename pcl::PointCloud<PointT>::Ptr &cloud, const float leafSize)
    {
        // The filtered point cloud object
        typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

        // Define the downsampling filter
        typename pcl::VoxelGrid<PointT>::Ptr downsampleFilter(new pcl::VoxelGrid<PointT>());

        // Set the parameters of the downsampling filter
        downsampleFilter->setLeafSize(leafSize, leafSize, leafSize);
        downsampleFilter->setInputCloud(cloud);

        // Apply the downsampling filter
        downsampleFilter->filter(*filtered);
        filtered->header = cloud->header;

        return filtered;
    }
    template pcl::PointCloud<pcl::PointXYZRGB>::Ptr Utils::pointcloudDownsample<pcl::PointXYZRGB>(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, const float);
    template pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Utils::pointcloudDownsample<pcl::PointXYZRGBNormal>(
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, const float);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Utils::pointcloudDistanceFilter(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::pair<float, float> thresholds)
    {
        const float thresholdNear = thresholds.first;
        const float thresholdFar = thresholds.second;
        double distance;

        // Define the filtered point cloud object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        filteredCloud->reserve(cloud->size());

        // Filter the point cloud
        std::copy_if(cloud->begin(),
                     cloud->end(),
                     std::back_inserter(filteredCloud->points),
                     [&](const pcl::PointXYZRGB &p)
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

    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> Utils::ransacPlaneFitting(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int minSegmentationPoints)
    {
        // Variables
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> extractedPlanes;

        // Loop over cloud points as long as the cloud is large enough
        // [TODO] Temporary disabling sequential ransac
        // while (cloud->points.size() > minSegmentationPoints)
        const uint8_t maxRansacIterations = 1;
        for (uint8_t i = 0; i < maxRansacIterations && cloud->points.size() > minSegmentationPoints; i++)
        {
            try
            {
                // Create objects for RANSAC plane segmentation
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
                // Create the SAC segmentation object
                pcl::SACSegmentation<pcl::PointXYZRGB> seg;

                // Fill the values of the segmentation object
                seg.setInputCloud(cloud);
                seg.setNumberOfThreads(8);
                seg.setMaxIterations(200);
                seg.setDistanceThreshold(0.1);
                seg.setOptimizeCoefficients(true);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setModelType(pcl::SACMODEL_PLANE);

                // Apply RANSAC segmentation
                seg.segment(*inliers, *coeffs);

                // Check if any model was found while processing the point cloud indices
                // if (inliers->indices.empty())
                //     break;

                // Calculate normal on the plane
                Eigen::Vector4d planeEquation(coeffs->values[0], coeffs->values[1],
                                              coeffs->values[2], coeffs->values[3]);

                // Calculate the closest points
                Eigen::Vector4d plane;
                Eigen::Vector3d closestPoint = planeEquation.head(3) * planeEquation(3);
                plane.head(3) = closestPoint / closestPoint.norm();
                plane(3) = closestPoint.norm();

                // Create a new point cloud containing the points of the detected planes
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr extractedCloud(
                    new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                for (const auto &idx : inliers->indices)
                {
                    pcl::PointXYZRGBNormal tmpCloud;
                    // Fill the point cloud
                    tmpCloud.x = cloud->points[idx].x;
                    tmpCloud.y = cloud->points[idx].y;
                    tmpCloud.z = cloud->points[idx].z;
                    tmpCloud.normal_x = plane(0);
                    tmpCloud.normal_y = plane(1);
                    tmpCloud.normal_z = plane(2);
                    tmpCloud.curvature = plane(3);
                    // Add the point to the cloud
                    extractedCloud->points.push_back(tmpCloud);
                }

                // Add the extracted cloud to the vector
                extractedPlanes.push_back(extractedCloud);

                // Extract the inliers
                extract.setInputCloud(cloud);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*cloud);
            }
            catch (const std::exception &e)
            {
                std::cout << "RANSAC model error!" << std::endl;
                // break;
            }
        }

        // Return the extracted clouds
        return extractedPlanes;
    }

    ORB_SLAM3::Plane::planeVariant Utils::getPlaneTypeFromClassId(int clsId)
    {
        switch (clsId)
        {
        case 0:
            return ORB_SLAM3::Plane::planeVariant::FLOOR;
        case 1:
            return ORB_SLAM3::Plane::planeVariant::WALL;
        default:
            return ORB_SLAM3::Plane::planeVariant::UNDEFINED;
        }
    }

    bool Utils::pointOnPlane(Eigen::Vector4d planeEquation, MapPoint *mapPoint)
    {
        if (mapPoint->isBad())
            return false;

        // Find the distance of the point from a given plane
        double pointPlaneDist =
            planeEquation(0) * mapPoint->GetWorldPos()(0) +
            planeEquation(1) * mapPoint->GetWorldPos()(1) +
            planeEquation(2) * mapPoint->GetWorldPos()(2) +
            planeEquation(3);

        // Apply a threshold
        if (fabs(pointPlaneDist) < 0.1)
            return true;

        return false;
    }

    int Utils::associatePlanes(const vector<Plane *> &mappedPlanes, g2o::Plane3D givenPlane)
    {
        int planeId = -1;

        // Initialize difference value
        double minDiff = 100.0;
        // Fixed threshold for comparing two planes
        double diffThreshold = 0.5;

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
        if (minDiff < diffThreshold)
            return planeId;

        // Otherwise, return -1 so that the the plane gets added to the map
        return -1;
    }

    double Utils::calcSoftMin(vector<double> &values)
    {
        // parameter controlling the softness/sharpness of the soft-min
        const double tau = 0.1;

        // soft-min = sum(exp(-value/tau) * value) / sum(exp(-value/tau))
        Eigen::Map<Eigen::VectorXd> confs(values.data(), values.size());
        Eigen::VectorXd term = ((1.0 - confs.array()) / tau).exp();
        return ((term / term.sum()).array() * confs.array()).sum();
    }
}