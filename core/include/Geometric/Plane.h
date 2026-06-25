/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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

#ifndef PLANE_H
#define PLANE_H

#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Marker.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"
#include "Types/SystemParams.h"
#include <set>

#include <boost/shared_ptr.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_search.h>

namespace ORB_SLAM3
{
class Map;
class Marker;
class MapPoint;

class Plane
{
  public:
    enum planeVariant
    {
        /**
         * @brief       TODO
         */
        UNDEFINED = -1,

        /**
         * @brief       TODO
         */
        WALL = 0,

        /**
         * @brief       TODO
         */
        GROUND = 1,

        /**
         * @brief       TODO
         */
        WINDOW = 2
    };

    /**
     * @brief       TODO
     */
    struct Observation
    {

        /**
         * @brief       The plane equation in the local frame
         */
        g2o::Plane3D localPlane;

        /**
         * @brief       The aggregated point cloud measurement for point-plane
         *              constraint
         */
        Eigen::Matrix4d Gij;

        /**
         * @brief       The aggregated confidence of the plane
         */
        double confidence;

        /**
         * @brief       The semantic type of the plane
         */
        planeVariant semanticType = UNDEFINED;
    };

    /* ---------------------------------------------------------------------- *
     * Variables for bundle adjustment
     * ---------------------------------------------------------------------- */

    /**
     * @brief       The first keyframe that observed the plane is the  reference
     *              keyframe
     */
    KeyFrame *refKeyFrame;

    /**
     * @brief       The reference keyframe ID for the Global BA the plane was
     *              part of
     */
    unsigned long int mnBAGlobalForKF;

    /**
     * @brief       The plane equation in the global map after the Global BA
     */
    g2o::Plane3D mPlaneGBA;

  private:
    /**
     * @brief       The plane's identifier
     */
    int id;

    /**
     * @brief       The plane's identifier in the local optimizer
     */
    int opId;

    /**
     * @brief       The plane's identifier in the global optimizer
     */
    int opIdG;

    /**
     * @brief       Marks the plane as bad (if true, the plane will not be used)
     */
    bool mbBad;

    /**
     * @brief       The plane's semantic type (e.g., wall, ground, etc.)s
     */
    planeVariant planeType;

    /**
     * @brief       The centroid of the plane
     */
    Eigen::Vector3f centroid;

    /**
     * @brief       A color devoted for visualization
     */
    std::vector<uint8_t> color;

    /**
     * @brief       The plane equation in the local map
     */
    g2o::Plane3D localEquation;

    /**
     * @brief       The plane equation in the global map
     */
    g2o::Plane3D globalEquation;

    /**
     * @brief       The unique set of map points lying on the plane
     */
    std::set<MapPoint *> mapPoints;

    /**
     * @brief       The votes for the semantic type of the plane
     */
    std::map<planeVariant, double> semanticVotes;

    /**
     * @brief       Plane's observations in keyFrames
     */
    std::map<KeyFrame *, Observation> observations; //

    /**
     * @brief       The point cloud of the plane
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;

    /**
     * @brief       The octree for the plane cloud
     */
    boost::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>>
        octree;

  public:
    Plane();
    ~Plane();

    /**
     * @brief       TODO
     */
    int getId() const;

    /**
     * @brief       TODO
     */
    void setId(int value);

    /**
     * @brief       TODO
     */
    int getOpId() const;

    /**
     * @brief       TODO
     */
    void setOpId(int value);

    /**
     * @brief       TODO
     */
    int getOpIdG() const;

    /**
     * @brief       TODO
     */
    void setOpIdG(int value);

    /**
     * @brief       TODO
     */
    bool isBad();

    /**
     * @brief       TODO
     */
    void setBad();

    /**
     * @brief       TODO
     */
    void setColor();

    /**
     * @brief       TODO
     */
    std::vector<uint8_t> getColor() const;

    /**
     * @brief       TODO
     */
    planeVariant getPlaneType();

    /**
     * @brief       TODO
     */
    planeVariant getExpectedPlaneType();

    /**
     * @brief       TODO
     */
    void setPlaneType(planeVariant newType);

    /**
     * @brief       TODO
     */
    void setMapPoints(MapPoint *value);

    /**
     * @brief       TODO
     */
    std::set<MapPoint *> getMapPoints();

    /**
     * @brief       TODO
     */
    Eigen::Vector3f getCentroid() const;

    /**
     * @brief       TODO
     */
    void setCentroid(const Eigen::Vector3f &value);

    /**
     * @brief       TODO
     */
    g2o::Plane3D getLocalEquation() const;

    /**
     * @brief       TODO
     */
    void setLocalEquation(const g2o::Plane3D &value);

    /**
     * @brief       TODO
     */
    g2o::Plane3D getGlobalEquation() const;

    /**
     * @brief       TODO
     */
    void setGlobalEquation(const g2o::Plane3D &value);

    /**
     * @brief       TODO
     */
    void addObservation(KeyFrame *pKF, Observation obs);

    /**
     * @brief       TODO
     */
    void eraseObservation(KeyFrame *pKF);

    /**
     * @brief       TODO
     */
    const std::map<KeyFrame *, Observation> &getObservations() const;

    /**
     * @brief       TODO
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getMapClouds();

    /**
     * @brief       TODO
     */
    void setMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr value);

    /**
     * @brief       TODO
     */
    void replaceMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr value);

    /**
     * @brief       TODO
     */
    bool isPointinPlaneCloud(const Eigen::Vector3d &point);

    /**
     * @brief       TODO
     */
    void castWeightedVote(planeVariant semanticType, double voteWeight);

    /**
     * @brief       TODO
     */
    void resetPlaneSemantics();

    /**
     * @brief       TODO
     */
    Map *GetMap();

    /**
     * @brief       TODO
     */
    void SetMap(Map *pMap);

  protected:
    /**
     * @brief       TODO
     */
    Map *mpMap;

    /**
     * @brief       TODO
     */
    std::mutex mMutexMap, mMutexType;

    /**
     * @brief       TODO
     */
    mutable std::mutex mMutexFeatures, mMutexPos;
};
} // namespace ORB_SLAM3

#endif