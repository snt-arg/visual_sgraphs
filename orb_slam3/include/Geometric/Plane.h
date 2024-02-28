/**
 * 🚀 [vS-Graphs] Planes Entity
 */

#ifndef PLANE_H
#define PLANE_H

#include <set>
#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Marker.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"

#include <pcl/common/io.h>

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
            UNDEFINED = -1,
            WALL = 0,
            FLOOR = 1,
            WINDOW = 2
        };

    private:
        int id;                                                  // The plane's identifier
        int opId;                                                // The plane's identifier in the local optimizer
        int opIdG;                                               // The plane's identifier in the global optimizer
        planeVariant planeType;                                  // The plane's semantic type (e.g., wall, floor, etc.)
        Eigen::Vector3f centroid;                                // The centroid of the plane
        std::vector<double> color;                               // A color devoted for visualization
        g2o::Plane3D localEquation;                              // The plane equation in the local map
        g2o::Plane3D globalEquation;                             // The plane equation in the global map
        std::vector<Marker *> markers;                           // The list of markers lying on the plane
        std::set<MapPoint *> mapPoints;                          // The unique set of map points lying on the plane
        std::map<planeVariant, double> semanticVotes;            // The votes for the semantic type of the plane
        std::map<KeyFrame *, g2o::Plane3D> observations;         // Plane's observations in keyFrames
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud; // The point cloud of the plane

    public:
        Plane();
        ~Plane();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        void setColor();
        std::vector<double> getColor() const;

        void setMarkers(Marker *value);
        std::vector<Marker *> getMarkers() const;

        planeVariant getPlaneType() const;
        void setPlaneType(planeVariant newType);

        void setMapPoints(MapPoint *value);
        std::set<MapPoint *> getMapPoints();

        Eigen::Vector3f getCentroid() const;
        void setCentroid(const Eigen::Vector3f &value);

        g2o::Plane3D getLocalEquation() const;
        void setLocalEquation(const g2o::Plane3D &value);

        g2o::Plane3D getGlobalEquation() const;
        void setGlobalEquation(const g2o::Plane3D &value);

        void addObservation(KeyFrame *pKF, g2o::Plane3D localEquation);
        const std::map<KeyFrame *, g2o::Plane3D> &getObservations() const;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getMapClouds() const;
        void setMapClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr value);
        void replaceMapClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr value);

        void updatePlaneType();
        void castWeightedVote(planeVariant semanticType, double voteWeight);

        Map *GetMap();
        void SetMap(Map *pMap);

    protected:
        Map *mpMap;
        std::mutex mMutexMap, mMutexPoint;
    };
}

#endif