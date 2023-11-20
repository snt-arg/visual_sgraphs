/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#include "Geometric/Plane.h"

namespace ORB_SLAM3
{
    Plane::Plane() {}
    Plane::~Plane() {}

    int Plane::getId() const
    {
        return id;
    }

    void Plane::setId(int value)
    {
        id = value;
    }

    int Plane::getOpId() const
    {
        return opId;
    }

    void Plane::setOpId(int value)
    {
        opId = value;
    }

    int Plane::getOpIdG() const
    {
        return opIdG;
    }

    void Plane::setOpIdG(int value)
    {
        opIdG = value;
    }

    std::vector<double> Plane::getColor() const
    {
        return color;
    }

    void Plane::setColor()
    {
        if (color.size() == 0)
        {
            color.push_back(rand() % 256);
            color.push_back(rand() % 256);
            color.push_back(rand() % 256);
        }
    }

    std::vector<Marker *> Plane::getMarkers() const
    {
        return markers;
    }

    void Plane::setMarkers(Marker *value)
    {
        // Check if the marker is not already added in the list of plane markers
        if (std::find(markers.begin(), markers.end(), value) == markers.end())
        {
            markers.push_back(value);
        }
    }

    std::set<MapPoint *> Plane::getMapPoints()
    {
        unique_lock<mutex> lock(mMutexPoint);
        return map_points;
    }

    void Plane::setMapPoints(MapPoint *value)
    {
        unique_lock<mutex> lock(mMutexPoint);
        map_points.insert(value);
    }

    g2o::Plane3D Plane::getEquation() const
    {
        return equation;
    }

    void Plane::setEquation(const g2o::Plane3D &value)
    {
        equation = value;
    }

    Eigen::Vector3f Plane::getCentroid() const
    {
        return centroid;
    }

    void Plane::setCentroid(const Eigen::Vector3f &value)
    {
        centroid = value;
    }

    Map *Plane::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Plane::SetMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}