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

#ifndef PASSAGE_H
#define PASSAGE_H

#include "Map.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"

namespace ORB_SLAM3
{
    class Map;
    class Plane;
    class Marker;

    class Passage
    {
    public:
        enum passageVariant
        {
            UNDEFINED = -1,
            DOORWAY = 0
        };

    private:
        int id;
        int opId;
        int opIdG;
        double width;
        double height;
        bool passable;
        Eigen::Vector3f centroid;
        passageVariant passageType;
        g2o::Plane3D globalEquation;
        ORB_SLAM3::Plane *associateDoor;
        std::vector<ORB_SLAM3::Plane *> associateWalls;

    public:
        Passage();
        ~Passage();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        bool isPassable() const;
        void setPassable(bool value);

        double getWidth() const;
        void setWidth(double value);

        double getHeight() const;
        void setHeight(double value);

        passageVariant getPassageType();
        void setPassageType(passageVariant newType);

        Eigen::Vector3f getCentroid() const;
        void setCentroid(const Eigen::Vector3f &value);

        g2o::Plane3D getGlobalEquation() const;
        void setGlobalEquation(const g2o::Plane3D &value);

        ORB_SLAM3::Plane *getAssociateDoor() const;
        void setAssociateDoor(ORB_SLAM3::Plane *value);

        void addAssociateWall(ORB_SLAM3::Plane *value);
        std::vector<ORB_SLAM3::Plane *> getAssociateWalls() const;

        ORB_SLAM3::Map *getMap();
        void setMap(ORB_SLAM3::Map *pMap);

    protected:
        ORB_SLAM3::Map *mpMap;
        std::mutex mMutexMap, mMutexType;
    };

}

#endif