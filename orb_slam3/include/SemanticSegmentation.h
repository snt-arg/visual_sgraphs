/**
 * 🚀 [vS-Graphs] A Class for Semantic Segmentation of the Scene
 */

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

namespace ORB_SLAM3
{
    class Atlas;

    class SemanticSegmentation
    {
    private:
        Atlas *mpAtlas;
        std::mutex mMutexNewKFs;
        double mSegProbThreshold;
        std::list<std::pair<cv::Mat, pcl::PCLPointCloud2::Ptr>> segmentedImageBuffer;
        const uint8_t bytesPerClassProb = 4; // 4 bytes per class probability - refer to scene_segment_ros

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticSegmentation(Atlas *pAtlas, double segProbThreshold);

        void AddSegmentedFrameToBuffer(std::pair<cv::Mat, pcl::PCLPointCloud2::Ptr> *pair);
        std::list<std::pair<cv::Mat, pcl::PCLPointCloud2::Ptr>> GetSegmentedFrameBuffer();
        void threshSeparatePointCloud(
            pcl::PCLPointCloud2::Ptr &pclPc2SegPrb, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clsCloudPtrs);

        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H