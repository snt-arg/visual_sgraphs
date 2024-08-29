/**
 * 🚀 [vS-Graphs] Door Frame Entity
 */

#include "Semantic/Door.h"

namespace ORB_SLAM3
{
    Door::Door() {}
    Door::~Door() {}

    int Door::getId() const
    {
        return id;
    }

    void Door::setId(int value)
    {
        id = value;
    }

    int Door::getOpId() const
    {
        return opId;
    }

    void Door::setOpId(int value)
    {
        opId = value;
    }

    int Door::getOpIdG() const
    {
        return opIdG;
    }

    void Door::setOpIdG(int value)
    {
        opIdG = value;
    }

    int Door::getMarkerId() const
    {
        return markerId;
    }

    void Door::setMarkerId(int value)
    {
        markerId = value;
    }

    std::string Door::getName() const
    {
        return name;
    }

    void Door::setName(std::string value)
    {
        name = value;
    }

    Marker *Door::getMarker() const
    {
        return marker;
    }

    void Door::setMarker(Marker *value)
    {
        marker = value;
    }

    Sophus::SE3f Door::getLocalPose() const
    {
        return localPose;
    }

    void Door::setLocalPose(const Sophus::SE3f &value)
    {
        localPose = value;
    }

    Sophus::SE3f Door::getGlobalPose() const
    {
        return globalPose;
    }

    void Door::setGlobalPose(const Sophus::SE3f &value)
    {
        globalPose = value;
    }

    Map *Door::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Door::setMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}