#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas)
    {
        mpAtlas = pAtlas;
    }

    void SemanticSegmentation::Run()
    {
        while (1)
        {
            // Check if there are new KeyFrames in the buffer
            if (segmentedImageBuffer.empty())
            {
                usleep(3000);
                continue;
            }

            // Get the unfiltered point cloud of the KF
            // Get the probability map for the pointcloud from Semantic Segmenter
            // For probabilities > 0.9, as the wall, segment all the points in the point cloud
            // Return the pointcloud with only walls and floors
            // Perform distance filter and downsample
            // Send it to the Ransac and get the plane equations
            // Do the association with the planes in Geometric
            usleep(3000);
        }
    }

    void SemanticSegmentation::AddSegmentedFrameToBuffer(std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *pair)
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        segmentedImageBuffer.push_back(*pair);
    }

    std::list<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> SemanticSegmentation::GetSegmentedFrameBuffer()
    {
        return segmentedImageBuffer;
    }
}