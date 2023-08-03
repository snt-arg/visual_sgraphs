/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef DBPARSER_H
#define DBPARSER_H

#include <fstream>
#include <iostream>
#include "Thirdparty/nlohmann/json.hpp"

#include "Semantic/Door.h"
#include "Semantic/Room.h"

using json = nlohmann::json;

namespace ORB_SLAM3
{
    /**
     * @brief This class functions to parse data extracted from JSON files.
     */
    class DBParser
    {
    private:
        std::vector<Room> envRooms; // Rooms available in the real environment
        std::vector<Door> envDoors; // Doors available in the real environment

    public:
        DBParser();
        ~DBParser();

        /**
         * @brief Parses the JSON file and returns a dictionary of its values.
         * @param jsonFilePath the path of the JSON file
         */
        json jsonParser(string jsonFilePath);

        /**
         * @brief Parses the dictionary containing rooms data in the real environment
         * and returns a list of rooms.
         * @param jsonFilePath the path of the dictionary
         */
        std::vector<Room> getEnvRooms(json envData);

        /**
         * @brief Parses the dictionary containing doors data in the real environment
         * and returns a list of doors.
         * @param jsonFilePath the path of the dictionary
         */
        std::vector<Door> getEnvDoors(json envData);
    };
}

#endif