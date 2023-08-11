/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef WALL_H
#define WALL_H

#include <Eigen/Eigen>

#include "Map.h"
#include "Marker.h"
#include "../MapPoint.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"

namespace ORB_SLAM3
{
    class Wall
    {
    private:
        int id;                             // The wall's identifier
        int opId;                           // The wall's identifier in the local optimizer
        int opIdG;                          // The wall's identifier in the global optimizer
        g2o::Plane3D plane_equation;        // The plane equation of the wall
        std::vector<Marker *> markers;      // The list of markers lying on the wall
        std::vector<MapPoint *> map_points; // The set of map points lying on the wall

    public:
        Wall();
        ~Wall();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        void setMarkers(Marker *value);
        std::vector<Marker *> getMarkers() const;

        void setMapPoints(MapPoint *value);
        std::vector<MapPoint *> getMapPoints() const;

        g2o::Plane3D getPlaneEquation() const;
        void setPlaneEquation(const g2o::Plane3D &value);

        Map *GetMap();
        void SetMap(Map *pMap);

    protected:
        Map *mpMap;
        std::mutex mMutexMap;
    };
}

#endif