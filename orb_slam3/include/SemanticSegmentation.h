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
#include <pcl/common/transforms.h>

namespace ORB_SLAM3
{
    class Atlas;

    class SemanticSegmentation
    {
    private:
        bool mGeoRuns;
        Atlas *mpAtlas;
        std::mutex mMutexNewKFs;
        std::mutex mMutexNewRooms;
        Eigen::Matrix4f mPlanePoseMat;       // The transformation matrix from ground plane to horizontal
        const uint8_t bytesPerClassProb = 4; // Four bytes per class probability - refer to scene_segment_ros
        // The updated segmented frame buffer
        std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> segmentedImageBuffer;
        unsigned long int mLastProcessedKeyFrameId = 0;
        // The latest skeleton cluster
        std::vector<std::vector<Eigen::Vector3d *>> latestSkeletonCluster;

        // System parameters
        SystemParams *sysParams;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticSegmentation(Atlas *pAtlas);

        // Semantic segmentation frame buffer processing
        std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> GetSegmentedFrameBuffer();
        void AddSegmentedFrameToBuffer(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple);

        // Skeleton cluster processing
        std::vector<std::vector<Eigen::Vector3d *>> GetLatestSkeletonCluster();
        void UpdateSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d *>> &skeletonClusterPoints);

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
         * @brief Creates a map plane from the estimated plane
         * @param pKF the current keyframe
         * @param estimatedPlane the estimated plane
         * @param clsId the class id
         * @param conf the confidence of the class prediction
         * @param planeCloud the plane point cloud
         */
        void createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane, int clsId,
                            double conf, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud);

        /**
         * @brief Updates the map plane
         * @param planeId the plane id
         * @param clsId the class id
         * @param confidence the confidence of the class predictions
         */
        void updatePlaneSemantics(int planeId, int clsId, double confidence);

        /**
         * @brief Updates the map plane
         * @param pKF the current keyframe
         * @param estimatedPlane the estimated plane
         * @param planeCloud the plane point cloud
         * @param planeId the plane id
         */
        void updateMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud, int planeId);

        /**
         * @brief Filters the wall planes to remove heavily tilted walls
         */
        void filterWallPlanes();

        /**
         * @brief Reassociates semantically classified planes if they get closer after optimization
         */
        void reAssociateSemanticPlanes();

        /**
         * @brief Filters the ground plane to remove points that are too far from the plane
         * @param groundPlane the main ground plane that is the reference
         */
        void filterGroundPlanes(Plane *groundPlane);

        /**
         * @brief Transforms the plane equation to the ground reference defined by mPlanePoseMat
         * @param planeEq the plane equation
         * @return the transformed plane equation
         */
        Eigen::Vector3f transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq);

        /**
         * @brief Gets the median height of a ground plane after transformation to referece by mPlanePoseMat
         * @param groundPlane the ground plane
         * @return the median height of the ground plane
         */
        float computeGroundPlaneHeight(Plane *groundPlane);

        /**
         * @brief Computes the transformation matrix from the ground plane to the horizontal (y-inverted)
         * @param plane the plane
         * @return the transformation matrix
         */
        Eigen::Matrix4f computePlaneToHorizontal(const Plane *plane);

        /**
         * @brief Converts mapped room candidates to rooms using geometric constraints
         * 🚧 [vS-Graphs v.2.0] This solution is not very reliable. It is recommended to use Voxblox version.
         */
        void updateMapRoomCandidateToRoomGeo(KeyFrame *pKF);

        /**
         * @brief Converts mapped room candidates to rooms using voxmap and freespace clusters
         */
        void updateMapRoomCandidateToRoomVoxblox();

        /**
         * @brief Converts mapped room candidates to rooms using a GNN
         * @param roomCandidate the address of the candidate room
         */
        void updateMapRoomCandidateToRoomGNN(Room *roomCandidate);

        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H