/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef ROOM_H
#define ROOM_H

#include "Wall.h"

using namespace std;

namespace ORB_SLAM3
{
    struct Room
    {
        // The identifier devoted for each room
        int id;
        // The identifier devoted for each room in the graph
        int opt_id;
        // The name devoted for each room in the graph (optional, for better identification)
        string name;
        // The set of marker-pairs attached in a room, e.g. [[1, 2], [3, 4]]
        vector<vector<int>> markers;
        // bool to check if room is already detected
        bool room_markers_detected;
        // The vector of detected walls
        vector<Wall> walls;
        // room center
        vector<double> roomCenter;
    };
}

#endif