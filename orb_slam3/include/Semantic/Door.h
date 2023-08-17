/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef DOOR_H
#define DOOR_H

#include <Eigen/Eigen>

#include "Map.h"
#include "Marker.h"

namespace ORB_SLAM3
{
    class Map;
    class Marker;

    class Door
    {
    private:
        int id;                   // The door's identifier
        int opId;                 // The door's identifier in the local optimizer
        int opIdG;                // The door's identifier in the global optimizer
        int marker_id;            // The marker attached to a door [in real map]
        Marker *marker;           // The marker attached on the door
        std::string name;         // The name devoted for each door (optional)
        Sophus::SE3f local_pose;  // Door's pose (position and orientation) in the Local Map
        Sophus::SE3f global_pose; // Door's pose (position and orientation) in the Global Map

    public:
        Door();
        ~Door();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        int getMarkerId() const;
        void setMarkerId(int value);

        std::string getName() const;
        void setName(std::string value);

        Marker *getMarker() const;
        void setMarker(Marker *value);

        Sophus::SE3f getLocalPose() const;
        void setLocalPose(const Sophus::SE3f &value);

        Sophus::SE3f getGlobalPose() const;
        void setGlobalPose(const Sophus::SE3f &value);

        Map *GetMap();
        void SetMap(Map *pMap);

    protected:
        Map *mpMap;
        std::mutex mMutexMap;
    };

}

#endif