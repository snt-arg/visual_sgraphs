/**
 * 🚀 [vS-Graphs] A Class for Semantic Segmentation of the Scene
 */

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"

namespace ORB_SLAM3
{
    class Atlas;

    class SemanticSegmentation
    {
    private:
        Atlas *mpAtlas;
        std::mutex mMutexNewKFs;
        std::list<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> segmentedImageBuffer;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticSegmentation(Atlas *pAtlas);

        void AddSegmentedFrameToBuffer(std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *pair);
        std::list<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> GetSegmentedFrameBuffer();

        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H