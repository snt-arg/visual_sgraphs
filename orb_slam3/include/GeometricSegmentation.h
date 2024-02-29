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
        std::mutex mMutexNewKFs;
        float mDownsampleLeafSize;
        std::list<KeyFrame *> mvpKeyFrameBuffer;
        std::vector<ORB_SLAM3::Door *> envDoors;
        std::vector<ORB_SLAM3::Room *> envRooms;
        std::pair<float, float> mDistFilterThreshold;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GeometricSegmentation(Atlas *pAtlas, bool hasDepthCloud, int minCloudSize,
                              std::pair<float, float> distFilterThreshold, float downsampleLeafSize);

        void AddKeyFrameToBuffer(KeyFrame *pKF);
        std::list<KeyFrame *> GetKeyFrameBuffer();

        /**
         * @brief Sets the environment doors and rooms fetched from the database
         * @param nEnvDoors the address of the environment doors
         * @param nEnvRooms the address of the environment rooms
         */
        void setEnvFetchedValues(std::vector<Door *> nEnvDoors, std::vector<Room *> nEnvRooms)
        {
            envDoors = nEnvDoors;
            envRooms = nEnvRooms;
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
         * @brief Calculation of the equation of the plane from marker pose
         * @param rotationMatrix the rotation matrix
         * @param translation the translation matrix
         */
        Eigen::Vector4d getPlaneEquationFromPose(const Eigen::Matrix3f &rotationMatrix,
                                                 const Eigen::Vector3f &translation);

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
         * @brief Creates a new room object (corridor or room) to be added to the map
         * @param matchedRoom the address of the room matched from the database
         * @param attachedMarker the address of the attached marker
         */
        void createMapRoomCandidate(ORB_SLAM3::Room *matchedRoom, ORB_SLAM3::Marker *attachedMarker);

        /**
         * @brief Updates an existing room object (corridor or room) to be added to the map
         * @param roomCandidate the address of the candidate room
         */
        void updateMapRoomCandidate(Room *roomCandidate);

        /**
         * @brief Uses the detected markers to detect and map semantic objects, e.g., planes and doors
         * @param pKF the current keyframe in which the detection took place
         * @param mvpMapMarkers the address of the detected markers
         */
        void markerSemanticDetectionAndMapping(ORB_SLAM3::KeyFrame *pKF,
                                               const std::vector<Marker *> &mvpMapMarkers,
                                               const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud);

        /**
         * @brief Checks for the association of a given room
         * @param detectedRoom the address of the detected room
         */
        Room *roomAssociation(const ORB_SLAM3::Room *detectedRoom);

        /**
         * @brief Runs the geometric segmentation thread
         */
        void Run();
    };
}

#endif // GEOMETRICSEG_H