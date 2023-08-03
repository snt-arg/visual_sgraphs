/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef ROOM_H
#define ROOM_H

#include "Wall.h"
#include "Thirdparty/g2o/g2o/types/vertex_plane.h"

namespace ORB_SLAM3
{
    class Room
    {
    private:
        int id;                                   // The room's identifier
        int opId;                                 // The room's identifier in the local optimizer
        int opIdG;                                // The room's identifier in the global optimizer
        std::string name;                         // The name devoted for each room (optional)
        bool all_markers_seen;                    // Checks if the room markers are already detected
        std::vector<Wall *> walls;                // The vector of detected walls of a room
        Eigen::Vector3d room_center;              // The center of the room as a 3D vector
        std::vector<std::vector<int>> marker_ids; // The set of marker-pairs attached to a room, e.g. [[1, 2], [3, 4]]

    public:
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

        bool getAllMarkersSeen() const;
        void setAllMarkersSeen(bool value);

        std::vector<Wall *> getWalls() const;
        void setWalls(std::vector<Wall *> &value);

        Eigen::Vector3d getRoomCenter() const;
        void setRoomCenter(Eigen::Vector3d value);

        std::vector<std::vector<int>> getMarkerIds() const;
        void setMarkerIds(std::vector<std::vector<int>> value);
    };
}

#endif