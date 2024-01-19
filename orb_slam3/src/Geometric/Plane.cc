/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#include "Geometric/Plane.h"

namespace ORB_SLAM3
{
    Plane::Plane()
    {
        plane_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    }
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

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Plane::getMapClouds()
    {
        unique_lock<mutex> lock(mMutexPoint);
        return plane_cloud;
    }

    void Plane::setMapClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr value)
    {
        unique_lock<mutex> lock(mMutexPoint);
        for (const auto &point : value->points)
            plane_cloud->points.push_back(point);
    }

    semanticType Plane::getPlaneType() const
    {
        return planeType;
    }

    void Plane::setPlaneType(semanticType newType)
    {
        planeType = newType;
    }

    g2o::Plane3D Plane::getLocalEquation() const
    {
        return local_equation;
    }

    void Plane::setLocalEquation(const g2o::Plane3D &value)
    {
        local_equation = value;
    }

    g2o::Plane3D Plane::getGlobalEquation() const
    {
        return global_equation;
    }

    void Plane::setGlobalEquation(const g2o::Plane3D &value)
    {
        global_equation = value;
    }

    Eigen::Vector3f Plane::getCentroid() const
    {
        return centroid;
    }

    void Plane::setCentroid(const Eigen::Vector3f &value)
    {
        centroid = value;
    }

    const std::map<KeyFrame *, g2o::Plane3D> &Plane::getObservations() const
    {
        return observations;
    }

    void Plane::addObservation(KeyFrame *pKF, g2o::Plane3D localEquation)
    {
        observations.insert({pKF, localEquation});
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