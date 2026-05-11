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

#include "Semantic/Passage.h"

namespace ORB_SLAM3
{
    Passage::Passage() : associateDoor(nullptr) {}
    Passage::~Passage() {}

    int Passage::getId() const
    {
        return id;
    }

    void Passage::setId(int value)
    {
        id = value;
    }

    int Passage::getOpId() const
    {
        return opId;
    }

    void Passage::setOpId(int value)
    {
        opId = value;
    }

    int Passage::getOpIdG() const
    {
        return opIdG;
    }

    void Passage::setOpIdG(int value)
    {
        opIdG = value;
    }

    bool Passage::isPassable() const
    {
        return passable;
    }

    void Passage::setPassable(bool value)
    {
        passable = value;
    }

    double Passage::getWidth() const
    {
        return width;
    }

    void Passage::setWidth(double value)
    {
        width = value;
    }

    double Passage::getHeight() const
    {
        return height;
    }

    void Passage::setHeight(double value)
    {
        height = value;
    }

    Passage::passageVariant Passage::getPassageType()
    {
        unique_lock<mutex> lock(mMutexType);
        return passageType;
    }

    void Passage::setPassageType(Passage::passageVariant newType)
    {
        unique_lock<mutex> lock(mMutexType);
        passageType = newType;
    }

    Eigen::Vector3f Passage::getCentroid() const
    {
        return centroid;
    }

    void Passage::setCentroid(const Eigen::Vector3f &value)
    {
        centroid = value;
    }

    ORB_SLAM3::Plane *Passage::getAssociateDoor() const
    {
        return associateDoor;
    }

    void Passage::setAssociateDoor(ORB_SLAM3::Plane *value)
    {
        associateDoor = value;
    }

    g2o::Plane3D Passage::getGlobalEquation() const
    {
        return globalEquation;
    }

    void Passage::setGlobalEquation(const g2o::Plane3D &value)
    {
        globalEquation = value;
    }

    std::vector<ORB_SLAM3::Plane *> Passage::getAssociateWalls() const
    {
        return associateWalls;
    }

    void Passage::addAssociateWall(ORB_SLAM3::Plane *value)
    {
        associateWalls.push_back(value);
    }

    ORB_SLAM3::Map *Passage::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Passage::setMap(ORB_SLAM3::Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}