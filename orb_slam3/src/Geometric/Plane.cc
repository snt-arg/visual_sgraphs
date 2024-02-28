/**
 * 🚀 [vS-Graphs] Planes Entity
 */

#include "Geometric/Plane.h"

namespace ORB_SLAM3
{
    Plane::Plane()
    {
        planeCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
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

    [[deprecated]] std::vector<Marker *> Plane::getMarkers() const
    {
        return markers;
    }

    [[deprecated]] void Plane::setMarkers(Marker *value)
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
        return mapPoints;
    }

    void Plane::setMapPoints(MapPoint *value)
    {
        unique_lock<mutex> lock(mMutexPoint);
        mapPoints.insert(value);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Plane::getMapClouds() const
    {
        return planeCloud;
    }

    void Plane::setMapClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr value)
    {
        unique_lock<mutex> lock(mMutexPoint);
        for (const auto &point : value->points)
            planeCloud->points.push_back(point);
    }

    void Plane::replaceMapClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr value)
    {
        unique_lock<mutex> lock(mMutexPoint);
        pcl::copyPointCloud(*value, *planeCloud);
    }

    Plane::planeVariant Plane::getPlaneType() const
    {
        return planeType;
    }

    void Plane::castWeightedVote(Plane::planeVariant semanticType, double voteWeight)
    {
        // check if semantic type is already in the semanticVotes map
        if (semanticVotes.find(semanticType) == semanticVotes.end())
            semanticVotes[semanticType] = voteWeight;
        else
            semanticVotes[semanticType] += voteWeight;

        // update based on new vote rankings
        updatePlaneType();
    }

    void Plane::updatePlaneType()
    {
        // find the semantic type with the maximum votes
        double maxVotes = 0;
        planeVariant maxType = planeVariant::UNDEFINED;
        for (const auto &vote : semanticVotes)
        {
            if (vote.second > maxVotes)
            {
                maxVotes = vote.second;
                maxType = vote.first;
            }
        }

        // set the plane type if votes above a certain threshold
        // [TODO] parameterize the maxVotes threshold
        if (maxVotes >= 2)
            planeType = maxType;
    }

    void Plane::setPlaneType(Plane::planeVariant newType)
    {
        planeType = newType;
    }

    g2o::Plane3D Plane::getLocalEquation() const
    {
        return localEquation;
    }

    void Plane::setLocalEquation(const g2o::Plane3D &value)
    {
        localEquation = value;
    }

    g2o::Plane3D Plane::getGlobalEquation() const
    {
        return globalEquation;
    }

    void Plane::setGlobalEquation(const g2o::Plane3D &value)
    {
        globalEquation = value;
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