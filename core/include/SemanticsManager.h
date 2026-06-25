/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#ifndef SEMANTICSMANAGER_H
#define SEMANTICSMANAGER_H

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Utils.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>

namespace ORB_SLAM3
{
class Atlas;

class SemanticsManager
{
  private:
    bool            mGeoRuns;
    Atlas          *mpAtlas;
    std::mutex      mMutexNewRooms;
    Eigen::Matrix4f mPlanePoseMat; // The transformation matrix from ground
                                   // plane to horizontal
    const uint8_t   runInterval =
        3; // The main Run() function runs every runInterval seconds

    // System parameters
    SystemParams *sysParams;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SemanticsManager(Atlas *pAtlas);

    /**
     * @brief       Gets the latest skeleton cluster acquired from voxblox
     */
    std::vector<std::vector<Eigen::Vector3d>> getLatestSkeletonCluster();

    /**
     * @brief       Gets the latest detected room candidates from GNN-based room
     *              detection
     */
    std::vector<ORB_SLAM3::Room *> getLatestGNNRoomCandidates();

    /**
     * @brief       Filters the wall planes to remove heavily tilted walls
     */
    void filterWallPlanes();

    /**
     * @brief       Filters the ground plane to remove points that are too far
     *              from the plane
     *
     * @param       groundPlane
     *              The main ground plane that is the reference
     */
    void filterGroundPlanes(Plane *groundPlane);

    /**
     * @brief       Transforms the plane equation to the ground reference
     *              defined by mPlanePoseMat
     *
     * @param       planeEq
     *              The plane equation
     *
     * @return      The transformed plane equation
     */
    Eigen::Vector3f
        transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq);

    /**
     * @brief       Gets the median height of a ground plane after
     *              transformation to referece by mPlanePoseMat
     *
     * @param       groundPlane
     *              The ground plane
     *
     * @return      The median height of the ground plane
     */
    float computeGroundPlaneHeight(Plane *groundPlane);

    /**
     * @brief       Computes the transformation matrix from the ground plane to
     *              the horizontal (y-inverted)
     *
     * @param       plane
     *              The plane
     *
     * @return      the transformation matrix
     */
    Eigen::Matrix4f computePlaneToHorizontal(const Plane *plane);

    /**
     * @brief       Gets the only rectangular room from the facing walls list
     *              (if exists, returns true)
     *
     * @param       givenRoom
     *              The address of the given room
     *
     * @param       facingWalls
     *              The facing walls list
     *
     * @param       perpThreshDeg
     *              The perpendicular threshold in degrees
     */
    bool getRectangularRoom(
        std::pair<std::pair<Plane *, Plane *>, std::pair<Plane *, Plane *>>
                                                       &givenRoom,
        const std::vector<std::pair<Plane *, Plane *>> &facingWalls,
        double                                          perpThreshDeg = 5.0);

    /**
     * @brief       Checks for the existing of a room with particular walls
     *              close to a cluster. It returns the address of the existing
     *              room if found, otherwise returns nullptr.
     *
     * @param       clusterCentroid
     *              The centroid of the cluster
     *
     * @param       wallList
     *              The list of walls to be checked
     */
    ORB_SLAM3::Room *
        associateRooms(const Eigen::Vector3d                  clusterCentroid,
                       const std::vector<ORB_SLAM3::Plane *> &wallList);

    /**
     * @brief       Re-associates rooms based on fixed time intervals to avoid
     *              duplicates
     */
    void reAssociateRooms();

    /**
     * @brief       Converts mapped room candidates to rooms using geometric
     *              constraints 🚧 [vS-Graphs v.1.2.0] This solution is not very
     *              reliable. It is recommended to use other structural element
     *              recognition solutions.
     */
    void updateMapRoomCandidateToRoomGeo(ORB_SLAM3::KeyFrame *pKF);

    /**
     * @brief       Processes the latest skeleton cluster to detect rooms based
     *              on free space clustering
     */
    void detectRoom_FreeSpaceCluster();

    /**
     * @brief       Gets the rooms detected by the GNN module
     */
    void detectRoom_GNN();

    /**
     * @brief       Gets the updated floors containing rooms and corridors
     */
    void getUpdatedFloors();

    // Running the thread
    void Run();
};
} // namespace ORB_SLAM3

#endif // SEMANTICSEG_H