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

    json DBParser::jsonParser(string jsonFilePath)
    {
        try
        {
            // Reading the JSON file from the given path
            ifstream jsonFile(jsonFilePath);
            // Parsing the JSON file to get the envrionment data
            json envData = json::parse(jsonFile);
            // Return parsed data
            return envData;
        }
        catch (json::parse_error &ex)
        {
            std::cerr << "Error while parsing the input JSON file: " << ex.byte << std::endl;
        }
    }

    std::vector<Room> DBParser::getEnvRooms(json envData)
    {
        envRooms.clear();
        return envRooms;
    }

    std::vector<Door> DBParser::getEnvDoors(json envData)
    {
        envDoors.clear();
        return envDoors;
    }
}