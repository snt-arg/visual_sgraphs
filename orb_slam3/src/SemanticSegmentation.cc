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
            std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clsPlanes =
                getPlanesFromClassClouds(clsCloudPtrs, minCloudSize);

            cout << "Number of Floor planes detected: " << clsPlanes[0].size() << endl;
            cout << "Number of Wall planes detected: " << clsPlanes[1].size() << endl;

            // [TODO] Associate the RANSACed planes with semantic classes in Atlas
            // [TODO] Associate semantic class ID with vector indices
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
                clsCloudPtrs[i]->points[j] = thisKFPointCloud->points[pointIndex];
            }
        }
    }

    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, int minCloudSize)
    {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clsPlanes;

        // downsample and filter the point clouds
        for (int i = 0; i < clsCloudPtrs.size(); i++)
        {
            // [TODO] decide when to downsample and/or distance filter

            // Downsample the given pointcloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud = Utils::pointcloudDownsample(clsCloudPtrs[i]);
            cout << "Downsampled cloud " << i << " has " << downsampledCloud->width << " points." << endl;

            // Filter the pointcloud based on a range of distance
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud = Utils::pointcloudDistanceFilter(downsampledCloud);
            cout << "Filtered cloud " << i << " has " << filteredCloud->width << " points." << endl;

            std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> extractedPlanes;
            if (filteredCloud->points.size() > minCloudSize)
            {
                //  Extract planes from the filtered point cloud
                extractedPlanes = Utils::ransacPlaneFitting(filteredCloud, minCloudSize);
            }
            clsPlanes.push_back(extractedPlanes);
        }
        return clsPlanes;
    }
}