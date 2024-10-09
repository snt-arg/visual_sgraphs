/**
 * 🚀 [vS-Graphs] A Class to manage the semantics inside the map
 */

#ifndef SEMANTICSMANAGER_H
#define SEMANTICSMANAGER_H

#include "Atlas.h"
#include "Utils.h"
#include "GeoSemHelpers.h"

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>

namespace ORB_SLAM3
{
    class Atlas;

    class SemanticsManager
    {
    private:
        bool mGeoRuns;
        Atlas *mpAtlas;
        std::mutex mMutexNewRooms;
        Eigen::Matrix4f mPlanePoseMat;       // The transformation matrix from ground plane to horizontal
        std::vector<std::vector<Eigen::Vector3d *>> latestSkeletonCluster;
        const uint8_t runInterval = 3;      // The main Run() function runs every runInterval seconds

        // System parameters
        SystemParams *sysParams;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticsManager(Atlas *pAtlas);

        /**
         * @brief Gets the latest skeleton cluster acquired from voxblox
         */
        std::vector<std::vector<Eigen::Vector3d *>> getLatestSkeletonCluster();

        /**
         * @brief Updates the skeleton cluster acquired from the current map (set by voxblox)
         */
        void setLatestSkeletonCluster();

        /**
         * @brief Filters the wall planes to remove heavily tilted walls
         */
        void filterWallPlanes();

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
         * @brief Gets the only rectangular room from the facing walls list (if exists, returns true)
         * @param givenRoom the address of the given room
         * @param facingWalls the facing walls list
         * @param perpThreshDeg the perpendicular threshold in degrees
         */
        bool getRectangularRoom(std::pair<std::pair<Plane *, Plane *>, std::pair<Plane *, Plane *>> &givenRoom,
                                const std::vector<std::pair<Plane *, Plane *>> &facingWalls,
                                double perpThreshDeg = 5.0);

        /**
         * @brief Checks for the association of a given room
         * @param givenRoom the address of the given room
         * @param givenRoomList the list of rooms to be checked
         */
        Room *roomAssociation(const ORB_SLAM3::Room *givenRoom, const vector<Room *> &givenRoomList);

        /**
         * @brief Converts mapped room candidates to rooms using geometric constraints
         * 🚧 [vS-Graphs v.2.0] This solution is not very reliable. It is recommended to use Voxblox version.
         */
        void updateMapRoomCandidateToRoomGeo(KeyFrame *pKF);

        /**
         * @brief Converts mapped room candidates to rooms using voxmap and freespace clusters
         */
        void detectMapRoomCandidateVoxblox();

        /**
         * @brief Converts mapped room candidates to rooms using a GNN
         */
        void detectMapRoomCandidateGNN();

        void removeBadMapPointsUsingSemantics();

        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H