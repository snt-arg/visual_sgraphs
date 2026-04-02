/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details: https://www.gnu.org/licenses/
 */

#include "Semantic/Room.h"

namespace ORB_SLAM3
{
    Room::Room()
    {
        metaMarker = nullptr;
    }
    Room::~Room() {}

    int Room::getId() const
    {
        return id;
    }

    void Room::setId(int value)
    {
        id = value;
    }

    int Room::getOpId() const
    {
        return opId;
    }

    void Room::setOpId(int value)
    {
        opId = value;
    }

    int Room::getOpIdG() const
    {
        return opIdG;
    }

    void Room::setOpIdG(int value)
    {
        opIdG = value;
    }

    void Room::setBad()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbBad = true;
    }

    bool Room::isBad()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbBad;
    }

    int Room::getMetaMarkerId() const
    {
        return metaMarkerId;
    }

    void Room::setMetaMarkerId(int value)
    {
        metaMarkerId = value;
    }

    Marker *Room::getMetaMarker() const
    {
        return metaMarker;
    }

    void Room::setMetaMarker(Marker *value)
    {
        metaMarker = value;
    }

    std::string Room::getName() const
    {
        return name;
    }

    void Room::setName(std::string value)
    {
        name = value;
    }

    Room::roomVariant Room::getRoomVariant()
    {
        return variant;
    }

    void Room::setRoomVariant(Room::roomVariant value)
    {
        variant = value;
    }

    bool Room::getHasKnownLabel() const
    {
        return hasKnownLabel;
    }

    void Room::setHasKnownLabel(bool value)
    {
        hasKnownLabel = value;
    }

    std::vector<Plane *> Room::getWalls() const
    {
        return walls;
    }

    void Room::setWalls(Plane *value)
    {
        walls.push_back(value);
    }

    void Room::clearWalls()
    {
        walls.clear();
    }

    Plane *Room::getGroundPlane() const
    {
        return groundPlane;
    }

    void Room::setGroundPlane(Plane *ground)
    {
        groundPlane = ground;
    }

    std::vector<ORB_SLAM3::Doorway *> Room::getDoorways() const
    {
        return doorways;
    }

    void Room::setDoorways(ORB_SLAM3::Doorway *value)
    {
        doorways.push_back(value);
    }

    Eigen::Vector3d Room::getCentroid() const
    {
        return centroid;
    }

    void Room::setCentroid(Eigen::Vector3d value)
    {
        centroid = value;
    }

    Map *Room::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Room::setMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}