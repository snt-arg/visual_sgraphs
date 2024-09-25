#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas)
    {
        mpAtlas = pAtlas;

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
            if (thisKF == nullptr || thisKF->isBad())
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

                // Compute the average confidence across all pixels in the plane observation
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = planePoint.first;
                std::vector<double> confidences;
                for (size_t i = 0; i < planeCloud->size(); i++)
                    confidences.push_back(static_cast<int>(planeCloud->points[i].a) / 255.0);
                double conf = Utils::calcSoftMin(confidences);

                // temp global plane cloud
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalPlaneCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::copyPointCloud(*planeCloud, *globalPlaneCloud);
                pcl::transformPointCloud(*globalPlaneCloud, *globalPlaneCloud, pKF->GetPoseInverse().matrix().cast<float>());

                // Check if we need to add the wall to the map or not
                int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(),
                                                            detectedPlane,
                                                            globalPlaneCloud,
                                                            pKF->GetPose().matrix().cast<double>(),
                                                            sysParams->seg.plane_association.ominus_thresh);

                // Get the semantic type of the observation
                ORB_SLAM3::Plane::planeVariant semanticType = Utils::getPlaneTypeFromClassId(clsId);

                if (matchedPlaneId == -1)
                {
                    if (!mGeoRuns)
                    {
                        ORB_SLAM3::Plane *newMapPlane = GeoSemHelpers::createMapPlane(mpAtlas, pKF, detectedPlane,
                                                                                      planeCloud, semanticType, conf);
                        // Cast a vote for the plane semantics
                        updatePlaneSemantics(newMapPlane->getId(), clsId, conf);
                    }
                }
                else
                {
                    if (!mGeoRuns)
                        GeoSemHelpers::updateMapPlane(mpAtlas, pKF, detectedPlane, planeCloud,
                                                      matchedPlaneId, semanticType, conf);
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
            }
        }
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
}