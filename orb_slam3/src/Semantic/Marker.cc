/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#include "Semantic/Marker.h"

namespace ORB_SLAM3
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
        return local_pose;
    }

    void Marker::setLocalPose(const Sophus::SE3f &value)
    {
        local_pose = value;
    }

    Sophus::SE3f Marker::getGlobalPose() const
    {
        return global_pose;
    }

    void Marker::setGlobalPose(const Sophus::SE3f &value)
    {
        global_pose = value;
    }

    const std::map<KeyFrame *, Sophus::SE3f> &Marker::getObservations() const
    {
        return mObservations;
    }

    void Marker::addObservation(KeyFrame *pKF, Sophus::SE3f local_pose)
    {
        mObservations.insert({pKF, local_pose});
    }

    Map *Marker::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Marker::SetMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
};