/**
 * 🚀 [vS-Graphs] Fiducial Marker Entity
 */

#include "Semantic/Marker.h"

namespace VS_GRAPHS
{
    Marker::Marker() {}
    Marker::~Marker() {}

    int Marker::getId() const
    {
        return id;
    }

    void Marker::setId(int value)
    {
        id = value;
    }

    int Marker::getOpId() const
    {
        return opId;
    }

    void Marker::setOpId(int value)
    {
        opId = value;
    }

    int Marker::getOpIdG() const
    {
        return opIdG;
    }

    void Marker::setOpIdG(int value)
    {
        opIdG = value;
    }

    double Marker::getTime() const
    {
        return time;
    }

    void Marker::setTime(double value)
    {
        time = value;
    }

    Marker::markerVariant Marker::getMarkerType() const
    {
        return markerType;
    }

    void Marker::setMarkerType(Marker::markerVariant newType)
    {
        markerType = newType;
    }

    bool Marker::isMarkerInGMap() const
    {
        return markerInGMap;
    }

    void Marker::setMarkerInGMap(bool value)
    {
        markerInGMap = value;
    }

    Sophus::SE3f Marker::getLocalPose() const
    {
        return localPose;
    }

    void Marker::setLocalPose(const Sophus::SE3f &value)
    {
        localPose = value;
    }

    Sophus::SE3f Marker::getGlobalPose() const
    {
        return globalPose;
    }

    void Marker::setGlobalPose(const Sophus::SE3f &value)
    {
        globalPose = value;
    }

    const std::map<KeyFrame *, Sophus::SE3f> &Marker::getObservations() const
    {
        return observations;
    }

    void Marker::addObservation(KeyFrame *pKF, Sophus::SE3f localPose)
    {
        observations.insert({pKF, localPose});
    }

    Map *Marker::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Marker::setMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
};