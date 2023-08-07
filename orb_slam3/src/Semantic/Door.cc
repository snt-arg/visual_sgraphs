/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
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
        return marker_id;
    }

    void Door::setMarkerId(int value)
    {
        marker_id = value;
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
        return local_pose;
    }

    void Door::setLocalPose(const Sophus::SE3f &value)
    {
        local_pose = value;
    }

    Sophus::SE3f Door::getGlobalPose() const
    {
        return global_pose;
    }

    void Door::setGlobalPose(const Sophus::SE3f &value)
    {
        global_pose = value;
    }

    Map *Door::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Door::SetMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}