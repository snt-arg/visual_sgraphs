/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details: https://www.gnu.org/licenses/
 */

#ifndef PLANE_H
#define PLANE_H

#include <set>
#include "Map.h"
#include "MapPoint.h"
#include "Types/SystemParams.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"

#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree_search.h>
#include <boost/shared_ptr.hpp>

namespace ORB_SLAM3
{
    class Map;
    class MapPoint;

    class Plane
    {
    public:
        enum planeVariant
        {
            UNDEFINED = -1,
            WALL = 0,
            GROUND = 1,
            DOOR = 2
        };

        struct Observation
        {
            g2o::Plane3D localPlane;               // The plane equation in the local frame
            Eigen::Matrix4d Gij;                   // The aggregated point cloud measurement for point-plane constraint
            double confidence;                     // The aggregated confidence of the plane
            planeVariant semanticType = UNDEFINED; // The semantic type of the plane
        };

        // Variables for bundle adjustment
        KeyFrame *refKeyFrame;             // The first keyframe that observed the plane is the reference keyframe
        unsigned long int mnBAGlobalForKF; // The reference keyframe ID for the Global BA the plane was part of
        g2o::Plane3D mPlaneGBA;            // The plane equation in the global map after the Global BA

    private:
        int id;                                                                           // The plane's identifier
        int opId;                                                                         // The plane's identifier in the local optimizer
        int opIdG;                                                                        // The plane's identifier in the global optimizer
        bool mbBad;                                                                       // Marks the plane as bad (if true, the plane will not be used)
        planeVariant planeType;                                                           // The plane's semantic type (e.g., wall, ground, door, etc.)
        Eigen::Vector3f centroid;                                                         // The centroid of the plane
        std::vector<uint8_t> color;                                                       // A color devoted for visualization
        g2o::Plane3D localEquation;                                                       // The plane equation in the local map
        g2o::Plane3D globalEquation;                                                      // The plane equation in the global map
        std::set<ORB_SLAM3::MapPoint *> mapPoints;                                        // The unique set of map points lying on the plane
        std::map<planeVariant, double> semanticVotes;                                     // The votes for the semantic type of the plane
        std::map<KeyFrame *, Observation> observations;                                   // Plane's observations in keyFrames
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;                               // The point cloud of the plane
        boost::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>> octree; // The octree for the plane cloud

    public:
        Plane();
        ~Plane();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        bool isBad();
        void setBad();

        void setColor();
        std::vector<uint8_t> getColor() const;

        planeVariant getPlaneType();
        planeVariant getExpectedPlaneType();
        void setPlaneType(planeVariant newType);

        void setMapPoints(ORB_SLAM3::MapPoint *value);
        std::set<ORB_SLAM3::MapPoint *> getMapPoints();

        Eigen::Vector3f getCentroid() const;
        void setCentroid(const Eigen::Vector3f &value);

        g2o::Plane3D getLocalEquation() const;
        void setLocalEquation(const g2o::Plane3D &value);

        g2o::Plane3D getGlobalEquation() const;
        void setGlobalEquation(const g2o::Plane3D &value);

        void addObservation(KeyFrame *pKF, Observation obs);
        void eraseObservation(KeyFrame *pKF);
        const std::map<KeyFrame *, Observation> &getObservations() const;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getMapClouds();
        void setMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr value);
        void replaceMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr value);
        bool isPointinPlaneCloud(const Eigen::Vector3d &point);

        void castWeightedVote(planeVariant semanticType, double voteWeight);
        void resetPlaneSemantics();

        ORB_SLAM3::Map *GetMap();
        void SetMap(ORB_SLAM3::Map *pMap);

    protected:
        ORB_SLAM3::Map *mpMap;
        std::mutex mMutexMap, mMutexType;
        mutable std::mutex mMutexFeatures, mMutexPos;
    };
}

#endif