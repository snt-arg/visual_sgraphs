/**
 * 🚀 [vS-Graphs] A Class for Semantic Segmentation of the Scene
 */

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"
#include "Utils.h"

#include <unordered_map>
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
        int mMinCloudSize;
        std::mutex mMutexNewKFs;
        double mSegProbThreshold;
        std::pair<float, float> mDistFilterThreshold;
        std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> segmentedImageBuffer;
        const uint8_t bytesPerClassProb = 4; // 4 bytes per class probability - refer to scene_segment_ros

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticSegmentation(Atlas *pAtlas, double segProbThreshold, int minCloudSize, std::pair<float, float> distFilterThreshold);

        std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> GetSegmentedFrameBuffer();
        void AddSegmentedFrameToBuffer(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple);

        /**
         * @brief Segments the point cloud into class specific point clouds
         * @param pclPc2SegPrb the point cloud to be segmented
         * @param clsCloudPtrs the class specific point clouds
         */
        void threshSeparatePointCloud(
            pcl::PCLPointCloud2::Ptr &pclPc2SegPrb, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs);

        /**
         * @brief Enriches the class-specific point clouds (with XYZ and RGB) with the current keyframe point cloud
         * @param clsCloudPtrs the class specific point clouds
         * @param thisKFPointCloud the current keyframe point cloud
         */
        void enrichClassSpecificPointClouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &thisKFPointCloud);

        /**
         * @brief Gets all planes for each class specific point cloud using RANSAC
         * @param clsCloudPtrs the class specific point clouds
         * @param minCloudSize the minimum size of the point cloud to be segmented
         * @return a vector of vector of point clouds
         */
        std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> getPlanesFromClassClouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, int minCloudSize);

        /**
         * @brief Adds the planes to the Atlas
         * @param clsPlanes the planes to be added
         */
        void addPlanesToAtlas(KeyFrame *pKF, std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> &clsPlanes);

        /**
         * @brief Creates a map plane from the estimated plane
         * @param pKF the current keyframe
         * @param estimatedPlane the estimated plane
         * @param clsId the class id
         * @param planeCloud the plane point cloud
         */
        void createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane, int clsId,
                            const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud);

        /**
         * @brief Updates the map plane
         * @param planeId the plane id
         * @param clsId the class id
         */
        void updateMapPlane(int planeId, int clsId);


        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H