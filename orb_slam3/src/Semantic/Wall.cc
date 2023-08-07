/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#include "Semantic/Wall.h"

namespace ORB_SLAM3
{
    Wall::Wall() {}
    Wall::~Wall() {}

    int Wall::getId() const
    {
        return id;
    }

    void Wall::setId(int value)
    {
        id = value;
    }

    int Wall::getOpId() const
    {
        return opId;
    }

    void Wall::setOpId(int value)
    {
        opId = value;
    }

    int Wall::getOpIdG() const
    {
        return opIdG;
    }

    void Wall::setOpIdG(int value)
    {
        opIdG = value;
    }

    g2o::VertexPlane *Wall::getWallPlane() const
    {
        return wall_plane;
    }

    void Wall::setWallPlane(g2o::VertexPlane *value)
    {
        wall_plane = value;
    }

    std::vector<Marker *> Wall::getMarkers() const
    {
        return markers;
    }

    void Wall::setMarkers(const std::vector<Marker *> &value)
    {
        markers = value;
    }

    std::vector<MapPoint *> Wall::getMapPoints() const
    {
        return map_points;
    }

    void Wall::setMapPoints(const std::vector<MapPoint *> &value)
    {
        map_points = value;
    }

    g2o::Plane3D Wall::getPlaneEquation() const
    {
        return plane_equation;
    }

    void Wall::setPlaneEquation(const g2o::Plane3D &value)
    {
        plane_equation = value;
    }

    Map *Wall::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Wall::SetMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}