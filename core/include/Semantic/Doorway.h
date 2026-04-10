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

#ifndef DOORWAY_H
#define DOORWAY_H

#include "Map.h"

namespace ORB_SLAM3
{
    class Map;
    class Plane;
    class Marker;

    class Doorway
    {
    private:
        int id;
        int opId;
        int opIdG;
        bool passable;
        Sophus::SE3f localPose;
        Sophus::SE3f globalPose;
        ORB_SLAM3::Plane *associateWall;

    public:
        Doorway();
        ~Doorway();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        bool isPassable() const;
        void setPassable(bool value);

        Sophus::SE3f getLocalPose() const;
        void setLocalPose(const Sophus::SE3f &value);

        Sophus::SE3f getGlobalPose() const;
        void setGlobalPose(const Sophus::SE3f &value);

        ORB_SLAM3::Plane *getAssociateWall() const;
        void setAssociateWall(ORB_SLAM3::Plane *value);

        ORB_SLAM3::Map *getMap();
        void setMap(ORB_SLAM3::Map *pMap);

    protected:
        ORB_SLAM3::Map *mpMap;
        std::mutex mMutexMap;
    };

}

#endif