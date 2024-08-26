#include "GeometricSegmentation.h"

namespace ORB_SLAM3
{
    GeometricSegmentation::GeometricSegmentation(Atlas *pAtlas, bool hasDepthCloud,
                                                 std::vector<ORB_SLAM3::Door *> nEnvDoors,
                                                 std::vector<ORB_SLAM3::Room *> nEvRooms)
    {
        mpAtlas = pAtlas;
        envRooms = nEvRooms;
        envDoors = nEnvDoors;
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
            g2o::Plane3D globalEquation = Utils::applyPoseToPlane(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                         detectedPlane);

            // convert planeCloud to global coordinates
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr emptyPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            
            // Check if we need to add the wall to the map or not
            int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);
            if (matchedPlaneId == -1)
                // A wall with the same equation was not found in the map, creating a new one
                GeoSemHelpers::createMapPlane(mpAtlas, pKF, detectedPlane, emptyPlaneCloud);
            else
                // The wall already exists in the map, fetching that one
                GeoSemHelpers::updateMapPlane(mpAtlas, pKF, detectedPlane, emptyPlaneCloud, matchedPlaneId);
        }

        // Add Markers while progressing in KFs
        GeoSemHelpers::markerSemanticAnalysis(mpAtlas, pKF, envDoors, envRooms);
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
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;
        filteredCloud = Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(cloudRGBA);
        filteredCloud = Utils::pointcloudDownsample<pcl::PointXYZRGBA>(filteredCloud,
                                                                       sysParams->sem_seg.pointcloud.downsample.leaf_size,
                                                                       sysParams->sem_seg.pointcloud.downsample.min_points_per_voxel);
        filteredCloud = Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(filteredCloud,
                                                                           sysParams->geo_seg.pointcloud.outlier_removal.std_threshold,
                                                                           sysParams->geo_seg.pointcloud.outlier_removal.mean_threshold);

        if (filteredCloud->points.size() > sysParams->seg.pointclouds_thresh)
            extractedPlanes = Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::SACSegmentation>(filteredCloud);

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

}