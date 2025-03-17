/**
 * 🚀 [vS-Graphs] Planes Entity
 */

#include "Geometric/Plane.h"

namespace VS_GRAPHS
{
    Plane::Plane()
    {
        planeCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
        octree = boost::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>>(SystemParams::GetParams()->refine_map_points.octree.resolution);
        centroid.setZero();
        excludedFromAssoc = false;
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

    std::vector<uint8_t> Plane::getColor() const
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

    std::set<MapPoint *> Plane::getMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mapPoints;
    }

    void Plane::setMapPoints(MapPoint *value)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mapPoints.insert(value);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Plane::getMapClouds()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return planeCloud;
    }

    void Plane::setMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr value)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        // store the old centroid for updating
        Eigen::Vector3f oldCentroid = getCentroid();
        oldCentroid *= planeCloud->points.size();

        // add the new points to the plane cloud
        for (const auto &point : value->points)
            planeCloud->points.push_back(point);

        // Update the plane centroid
        for (const auto &point : value->points)
            oldCentroid += Eigen::Vector3f(point.x, point.y, point.z);
        setCentroid(oldCentroid / planeCloud->points.size());

        // Update the octree
        octree->setInputCloud(planeCloud);
        octree->addPointsFromInputCloud();
    }

    void Plane::replaceMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr value)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        planeCloud->clear();
        pcl::copyPointCloud(*value, *planeCloud);

        // Update the plane centroid
        Eigen::Vector3f newCentroid;
        newCentroid.setZero();
        for (const auto &point : planeCloud->points)
            newCentroid += Eigen::Vector3f(point.x, point.y, point.z);
        setCentroid(newCentroid / planeCloud->points.size());

        // Update the octree
        octree->setInputCloud(planeCloud);
        octree->addPointsFromInputCloud();
    }

    bool Plane::isPointinPlaneCloud(const Eigen::Vector3d &point)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        pcl::PointXYZRGBA pointPCL;
        pointPCL.x = point(0);
        pointPCL.y = point(1);
        pointPCL.z = point(2);

        SystemParams *sysParams = SystemParams::GetParams();
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (octree->radiusSearch(pointPCL,
                                 sysParams->refine_map_points.octree.search_radius,
                                 pointIdxRadiusSearch,
                                 pointRadiusSquaredDistance,
                                 sysParams->refine_map_points.octree.min_neighbors) == sysParams->refine_map_points.octree.min_neighbors)
            return true;
        return false;
    }

    Plane::planeVariant Plane::getPlaneType()
    {
        unique_lock<mutex> lock(mMutexType);
        return planeType;
    }

    Plane::planeVariant Plane::getExpectedPlaneType()
    {
        unique_lock<mutex> lock(mMutexType);

        // get the maximum vote
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
        return maxType;
    }

    void Plane::castWeightedVote(Plane::planeVariant semanticType, double voteWeight)
    {
        unique_lock<mutex> lock(mMutexType);

        if (semanticType == planeVariant::UNDEFINED)
            return;

        // check if semantic type is already in the semanticVotes map
        if (semanticVotes.find(semanticType) == semanticVotes.end())
            semanticVotes[semanticType] = voteWeight;
        else
            semanticVotes[semanticType] += voteWeight;

        // update based on new vote rankings
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
        if (maxVotes >= SystemParams::GetParams()->sem_seg.min_votes)
            planeType = maxType;
        else
            planeType = planeVariant::UNDEFINED;
    }

    void Plane::setPlaneType(planeVariant newType)
    {
        unique_lock<mutex> lock(mMutexType);
        planeType = newType;
    }

    void Plane::resetPlaneSemantics()
    {
        unique_lock<mutex> lock(mMutexType);
        unique_lock<mutex> lock2(mMutexFeatures);
        semanticVotes.clear();
        planeType = planeVariant::UNDEFINED;
        planeCloud->clear();
        octree->deleteTree();
        observations.clear();
    }

    g2o::Plane3D Plane::getLocalEquation() const
    {
        unique_lock<mutex> lock(mMutexPos);
        return localEquation;
    }

    void Plane::setLocalEquation(const g2o::Plane3D &value)
    {
        unique_lock<mutex> lock(mMutexPos);
        localEquation = value;
    }

    g2o::Plane3D Plane::getGlobalEquation() const
    {
        unique_lock<mutex> lock(mMutexPos);
        return globalEquation;
    }

    void Plane::setGlobalEquation(const g2o::Plane3D &value)
    {
        unique_lock<mutex> lock(mMutexPos);
        globalEquation = value;
    }

    Eigen::Vector3f Plane::getCentroid() const
    {
        unique_lock<mutex> lock(mMutexPos);
        return centroid;
    }

    void Plane::setCentroid(const Eigen::Vector3f &value)
    {
        unique_lock<mutex> lock(mMutexPos);
        centroid = value;
    }

    const std::map<KeyFrame *, Plane::Observation> &Plane::getObservations() const
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return observations;
    }

    void Plane::addObservation(KeyFrame *pKF, Plane::Observation obs)
    {
        if (pKF == nullptr || pKF->isBad())
            return;
        unique_lock<mutex> lock(mMutexFeatures);
        observations.insert({pKF, obs});
    }

    void Plane::eraseObservation(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        Observation obs = observations[pKF];

        // remove the vote casted
        castWeightedVote(obs.semanticType, -obs.confidence);

        // remove the observation
        observations.erase(pKF);
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