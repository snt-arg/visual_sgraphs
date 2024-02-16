#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas, double segProbThreshold, int minCloudSize)
    {
        mpAtlas = pAtlas;
        mMinCloudSize = minCloudSize;
        mSegProbThreshold = segProbThreshold;
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
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clsCloudPtrs;
            threshSeparatePointCloud(pclPc2SegPrb, clsCloudPtrs);

            // get the point cloud from the respective keyframe via the atlas - ignore it if KF doesn't exist
            KeyFrame *thisKF = mpAtlas->GetKeyFrameById(std::get<0>(segImgTuple));
            if (thisKF == nullptr)
                continue;
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud = thisKF->getCurrentFramePointCloud();

            // fill in class specific point clouds with XYZZ and RGB from the keyframe pointcloud
            enrichClassSpecificPointClouds(clsCloudPtrs, thisKFPointCloud);

            // [TODO] make minCloudSize a parameter
            const int minCloudSize = 200;

            // get all planes for each class specific point cloud using RANSAC
            std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clsPlanes = 
                getPlanesFromClassClouds(clsCloudPtrs, minCloudSize);

            cout << "Floor planes detected: " << clsPlanes[0].size() << endl;
            cout << "Wall planes detected: " << clsPlanes[1].size() << endl;

            // Add the planes to Atlas
            addPlanesToAtlas(thisKF, clsPlanes);
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

    void SemanticSegmentation::threshSeparatePointCloud(
        pcl::PCLPointCloud2::Ptr &pclPc2SegPrb, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs)
    {
        const int width = pclPc2SegPrb->width;
        const int numPoints = width * pclPc2SegPrb->height;
        const int pointStep = pclPc2SegPrb->point_step;
        const int numClasses = pointStep / bytesPerClassProb;

        for (int i = 0; i < numClasses; i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pointCloud->is_dense = false;
            pointCloud->height = 1;
            clsCloudPtrs.push_back(pointCloud);
        }

        // parse the point cloud message and apply thresholding
        const uint8_t *data = pclPc2SegPrb->data.data();
        for (int i = 0; i < numPoints; i++)
        {
            for (int j = 0; j < numClasses; j++)
            {
                float value;
                memcpy(&value, data + pointStep * i + bytesPerClassProb * j + pclPc2SegPrb->fields[0].offset, bytesPerClassProb);

                // [TODO] make the thresholding a configuration parameter
                if (value >= 0.9)
                {
                    // inject coordinates as a point to respective point cloud
                    pcl::PointXYZRGB point;
                    point.y = static_cast<int>(i / width);
                    point.x = i % width;
                    clsCloudPtrs[j]->push_back(point);
                }
            }
        }

        // specify size/width and log statistics
        for (int i = 0; i < numClasses; i++)
        {
            clsCloudPtrs[i]->width = clsCloudPtrs[i]->size();
        }
    }

    void SemanticSegmentation::enrichClassSpecificPointClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &thisKFPointCloud)
    {
        for (int i = 0; i < clsCloudPtrs.size(); i++)
        {
            for (int j = 0; j < clsCloudPtrs[i]->width; j++)
            {
                const int pointIndex = clsCloudPtrs[i]->points[j].y * thisKFPointCloud->width + clsCloudPtrs[i]->points[j].x;
                
                // copy complete point since the segmented pointcloud has x, y in pixel coordinates
                clsCloudPtrs[i]->points[j] = thisKFPointCloud->points[pointIndex];
            }
        }
    }

    std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, int minCloudSize)
    {
        std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clsPlanes;

        // downsample/filter the pointcloud and extract planes
        for (int i = 0; i < clsCloudPtrs.size(); i++)
        {
            // [TODO] decide when to downsample and/or distance filter

            // Downsample the given pointcloud
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud = Utils::pointcloudDownsample(clsCloudPtrs[i]);

            // Filter the pointcloud based on a range of distance
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud = Utils::pointcloudDistanceFilter(clsCloudPtrs[i]);
            cout << "Downsampled/filtered cloud " << i << " has " << filteredCloud->width << " points." << endl;

            std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> extractedPlanes;
            if (filteredCloud->points.size() > minCloudSize)
            {
                extractedPlanes = Utils::ransacPlaneFitting(filteredCloud, minCloudSize);
            }
            clsPlanes[i] = extractedPlanes;
        }
        return clsPlanes;
    }

    void SemanticSegmentation::addPlanesToAtlas(KeyFrame *pKF,
                                                std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> &clsPlanes)
    {
        for (int clsId = 0; clsId < clsPlanes.size(); clsId++){
            for (auto planePoint : clsPlanes[clsId]){
                // Get the plane equation from the points
                Eigen::Vector4d planeEstimate(planePoint->back().normal_x, planePoint->back().normal_y,
                                            planePoint->back().normal_z, planePoint->back().curvature);
                g2o::Plane3D detectedPlane(planeEstimate);
                // Convert the given plane to global coordinates
                g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                            detectedPlane);

                // Check if we need to add the wall to the map or not
                int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);
                
                // [TODO] - Add flags to decide whether SemSeg can add planes or it just updates 
                if (matchedPlaneId == -1){
                    // [TODO] - Add a flag to decide whether SemSeg runs indepedently or with GeoSeg
                    cout << "No matched plane found." << endl;
                }
                else{
                    cout << "Matched a wall! Updating the plane type to " << clsId << endl;
                    updateMapPlane(matchedPlaneId, clsId);
                }
            }
        }
    }

    void SemanticSegmentation::createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane, int clsId,
                                               const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud)
    {
        ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->addObservation(pKF, estimatedPlane);
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());

        // Set the plane type to the semantic class
        newMapPlane->setPlaneType(Utils::getPlaneTypeFromClassId(clsId));

        // Set the global equation of the plane
        g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                     estimatedPlane);
        newMapPlane->setGlobalEquation(globalEquation);

        // Fill the plane with the pointcloud
        if (!planeCloud->points.empty())
            newMapPlane->setMapClouds(planeCloud);
        else
            // Loop to find the points lyinupdateMapPlane(pKF, detectedPlane, planePog on wall
            for (const auto &mapPoint : mpAtlas->GetAllMapPoints())
                if (Utils::pointOnPlane(newMapPlane->getGlobalEquation().coeffs(), mapPoint))
                    newMapPlane->setMapPoints(mapPoint);

        pKF->AddMapPlane(newMapPlane);
        mpAtlas->AddMapPlane(newMapPlane);
    }

    void SemanticSegmentation::updateMapPlane(int planeId, int clsId){
        Plane *matchedPlane = mpAtlas->GetPlaneById(planeId);
        matchedPlane->setPlaneType(Utils::getPlaneTypeFromClassId(clsId));      
    }
}