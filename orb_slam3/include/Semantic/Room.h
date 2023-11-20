/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef ROOM_H
#define ROOM_H

#include "Door.h"
#include "../Geometric/Plane.h"
#include "Thirdparty/g2o/g2o/types/vertex_plane.h"

namespace ORB_SLAM3
{
    class Door;
    class Plane;

    class Room
    {
    private:
        int id;                                        // The room's identifier
        int opId;                                      // The room's identifier in the local optimizer
        int opIdG;                                     // The room's identifier in the global optimizer
        std::string name;                              // The name devoted for each room (optional)
        bool all_seen_markers;                         // Checks if the room markers are already detected
        std::vector<Door *> doors;                     // The vector of detected doors of a room
        std::vector<Plane *> walls;                    // The vector of detected walls of a room
        Eigen::Vector3d room_center;                   // The center of the room as a 3D vector
        std::vector<int> door_marker_ids;              // Markers attached to the doors of a room [in real map], e.g. [3, 4]
        std::vector<std::vector<int>> wall_marker_ids; // Marker-pairs attached to a room [in real map], e.g. [[1, 2], [3, 4]]

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Room();
        ~Room();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        std::string getName() const;
        void setName(std::string value);

        bool getAllSeenMarkers() const;
        void setAllSeenMarkers(bool value);

        void setDoors(Door *value);
        std::vector<Door *> getDoors() const;

        void setWalls(Plane *value);
        std::vector<Plane *> getWalls() const;

        void clearWalls();

        void setDoorMarkerIds(int value);
        std::vector<int> getDoorMarkerIds() const;

        Eigen::Vector3d getRoomCenter() const;
        void setRoomCenter(Eigen::Vector3d value);

        void setWallMarkerIds(std::vector<int> value);
        std::vector<std::vector<int>> getWallMarkerIds() const;

        Map *GetMap();
        void SetMap(Map *pMap);

    protected:
        Map *mpMap;
        std::mutex mMutexMap;
    };
}

#endif