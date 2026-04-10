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

#include "Semantic/Doorway.h"

namespace ORB_SLAM3
{
    Doorway::Doorway() {}
    Doorway::~Doorway() {}

    int Doorway::getId() const
    {
        return id;
    }

    void Doorway::setId(int value)
    {
        id = value;
    }

    int Doorway::getOpId() const
    {
        return opId;
    }

    void Doorway::setOpId(int value)
    {
        opId = value;
    }

    int Doorway::getOpIdG() const
    {
        return opIdG;
    }

    void Doorway::setOpIdG(int value)
    {
        opIdG = value;
    }

    bool Doorway::isPassable() const
    {
        return passable;
    }

    void Doorway::setPassable(bool value)
    {
        passable = value;
    }

    double Doorway::getWidth() const
    {
        return width;
    }

    void Doorway::setWidth(double value)
    {
        width = value;
    }

    double Doorway::getHeight() const
    {
        return height;
    }

    void Doorway::setHeight(double value)
    {
        height = value;
    }

    Eigen::Vector3f Doorway::getCentroid() const
    {
        return centroid;
    }

    void Doorway::setCentroid(const Eigen::Vector3f &value)
    {
        centroid = value;
    }

    ORB_SLAM3::Plane *Doorway::getAssociateDoor() const
    {
        return associateDoor;
    }

    void Doorway::setAssociateDoor(ORB_SLAM3::Plane *value)
    {
        associateDoor = value;
    }

    g2o::Plane3D Doorway::getGlobalEquation() const
    {
        return globalEquation;
    }

    void Doorway::setGlobalEquation(const g2o::Plane3D &value)
    {
        globalEquation = value;
    }

    std::vector<ORB_SLAM3::Plane *> Doorway::getAssociateWalls() const
    {
        return associateWalls;
    }

    void Doorway::addAssociateWall(ORB_SLAM3::Plane *value)
    {
        associateWalls.push_back(value);
    }

    ORB_SLAM3::Map *Doorway::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Doorway::setMap(ORB_SLAM3::Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}