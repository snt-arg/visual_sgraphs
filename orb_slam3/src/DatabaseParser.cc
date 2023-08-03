/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#include "DatabaseParser.h"

namespace ORB_SLAM3
{
    DBParser::DBParser() {}
    DBParser::~DBParser() {}

    std::vector<Room> DBParser::getEnvRooms(string jsonFileName, bool absolutePath)
    {
        envRooms.clear();
        return envRooms;
    }

    std::vector<Door> DBParser::getEnvDoors(string jsonFileName, bool absolutePath)
    {
        envDoors.clear();
        return envDoors;
    }
}