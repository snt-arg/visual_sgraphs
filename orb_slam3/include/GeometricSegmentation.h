/**
 * 🚀 [vS-Graphs] A Class for Geometric Segmentation of the Scene
 */

#ifndef GEOMETRICSEG_H
#define GEOMETRICSEG_H

#include "Atlas.h"
#include "Utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

namespace ORB_SLAM3
{
    class Atlas;

    class GeometricSegmentation
    {
    private:
        Atlas *mpAtlas;
        int mMinCloudSize;
        bool mHasDepthCloud;
        float mDownsampleLeafSize;
        std::pair<float, float> mDistFilterThreshold;
        std::mutex mMutexNewKFs;
        std::list<KeyFrame *> mvpKeyFrameBuffer;
        std::vector<ORB_SLAM3::Door *> envDoors;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GeometricSegmentation(Atlas *pAtlas, bool hasDepthCloud, int minCloudSize,
                              std::pair<float, float> distFilterThreshold, float downsampleLeafSize);

        void AddKeyFrameToBuffer(KeyFrame *pKF);
        std::list<KeyFrame *> GetKeyFrameBuffer();

        /**
         * @brief Sets the environment doors fetched from the database
         * @param nEnvDoors the address of the environment doors
         */
        void setEnvDoors(std::vector<Door *> nEnvDoors)
        {
            envDoors = nEnvDoors;
        }

        /**
         * @brief Detects all the planes in the current keyframe
         * @param pKF the address of the current keyframe
         * @param hasDepthCloud a boolean to indicate if the point cloud has depth information
         * @param minCloudSize the minimum size of the point cloud to be segmented
         */
        void fetchPlanesFromKeyFrame(ORB_SLAM3::KeyFrame *pKF, bool hasDepthCloud, int minCloudSize);

        /**
         * @brief Calculation of plane equation from point clouds (provided by depth in RGB-D or calculated from
         * map points in Monocular and Stereo)
         * @param pKF the current KeyFrame
         * @param hasDepthCloud a boolean to indicate if the point cloud has depth information
         * @param minCloudSize the minimum size of the point cloud to be segmented
         */
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> getPlanesFromPointClouds(ORB_SLAM3::KeyFrame *pKF,
                                                                                           bool hasDepthCloud,
                                                                                           int minCloudSize = 200);

        /**
         * @brief Get the point cloud from a set of map-points
         * @param points the set of map-points
         */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudFromSparsePoints(const std::vector<MapPoint *> &points);

        /**
         * @brief Associates a detected plane into the planes found in the map and returns
         * if it needs to be added to the plane or not.
         * @param mappedPlanes an array of planes with their IDs and equations
         * @param givenPlane the detected 3D plane
         */
        int associatePlanes(const vector<Plane *> &mappedPlanes, g2o::Plane3D givenPlane);

        /**
         * @brief Calculation of the equation of the plane from marker pose
         * @param rotationMatrix the rotation matrix
         * @param translation the translation matrix
         */
        Eigen::Vector4d getPlaneEquationFromPose(const Eigen::Matrix3f &rotationMatrix,
                                                 const Eigen::Vector3f &translation);

        /**
         * @brief Finds the point lying on plane
         * @param planeEquation equation of a given plane
         * @param mapPoint current map point
         */
        bool pointOnPlane(Eigen::Vector4d planeEquation, MapPoint *mapPoint);

        /**
         * @brief Creates a new marker object to be added to the map
         * @param visitedMarker the address of the visited marker
         * @param pKF the address of the current keyframe
         */
        Marker *createMapMarker(const Marker *visitedMarker, KeyFrame *pKF);

        /**
         * @brief Creates a new plane object to be added to the map
         * @param planePointPair all the map points to check the ones lying on the plane
         * @param pKF the address of the current keyframe
         */
        void createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                            const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud);

        /**
         * @brief Updates an existing plane object in the map
         * @param planeId the identifier of the existing plane
         * @param visitedMarker the address of the visited marker
         * @param pKF the address of the current keyframe
         */
        void updateMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane,
                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud, int planeId,
                            ORB_SLAM3::Marker *visitedMarker = NULL);

        /**
         * @brief Creates a new door object to be added to the map
         * @param attachedMarker the address of the attached marker
         * @param pKF the address of the current keyframe
         * @param name the name of the door
         */
        void createMapDoor(Marker *attachedMarker, KeyFrame *pKF, std::string name);

        /**
         * @brief Uses the detected markers to detect and map semantic objects, e.g., planes and doors
         * @param pKF the current keyframe in which the detection took place
         * @param mvpMapMarkers the address of the detected markers
         */
        void markerSemanticDetectionAndMapping(ORB_SLAM3::KeyFrame *pKF,
                                               const std::vector<Marker *> &mvpMapMarkers,
                                               const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud);

        /**
         * @brief Runs the geometric segmentation thread
         */
        void Run();
    };
}

#endif // GEOMETRICSEG_H