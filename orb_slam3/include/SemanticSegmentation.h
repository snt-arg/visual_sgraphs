/**
 * 🚀 [vS-Graphs] A Class for Semantic Segmentation of the Scene
 */

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"
#include "Utils.h"
#include "GeoSemHelpers.h"

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>

namespace VS_GRAPHS
{
    class Atlas;

    class SemanticSegmentation
    {
    private:
        bool mGeoRuns;
        Atlas *mpAtlas;
        std::mutex mMutexNewKFs;
        const uint8_t bytesPerClassProb = 4; // Four bytes per class probability - refer to scene_segment_ros
        unsigned long int mLastProcessedKeyFrameId = 0;
        std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> segmentedImageBuffer;

        // System parameters
        SystemParams *sysParams;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticSegmentation(Atlas *pAtlas);

        // Semantic segmentation frame buffer processing
        std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> GetSegmentedFrameBuffer();
        void AddSegmentedFrameToBuffer(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple);

        /**
         * @brief Segments the point cloud into class specific point clouds and enriches them with the current keyframe point cloud
         * @param pclPc2SegPrb the point cloud to be segmented
         * @param segImgUncertainity the segmentation image uncertainty
         * @param clsCloudPtrs the class specific point clouds
         * @param thisKFPointCloud the current keyframe point cloud
         */
        void threshSeparatePointCloud(pcl::PCLPointCloud2::Ptr pclPc2SegPrb,
                                      cv::Mat &segImgUncertainity,
                                      std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs,
                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &thisKFPointCloud);

        /**
         * @brief Gets all planes for each class specific point cloud using RANSAC
         * @param clsCloudPtrs the class specific point clouds
         * @param minCloudSize the minimum size of the point cloud to be segmented
         * @return a vector of vector of point clouds
         */
        std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> getPlanesFromClassClouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs);

        /**
         * @brief Adds the planes to the Atlas
         * @param clsPlanes the planes to be added
         * @param clsConfs the confidence of the class predictions
         */
        void updatePlaneData(KeyFrame *pKF,
                             std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>> &clsPlanes);

        /**
         * @brief Updates the map plane
         * @param planeId the plane id
         * @param clsId the class id
         * @param confidence the confidence of the class predictions
         */
        void updatePlaneSemantics(int planeId, int clsId, double confidence);

        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H