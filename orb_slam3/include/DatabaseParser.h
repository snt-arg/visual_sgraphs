/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef DBPARSER_H
#define DBPARSER_H

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
         * @brief Parses the JSON file containing rooms data in the real environment
         * and returns a list of rooms.
         * @param jsonFileName the path of the JSON file
         * @param absolutePath is the absolute path given or not
         */
        std::vector<Room> getEnvRooms(string jsonFileName, bool absolutePath = false);

        /**
         * @brief Parses the JSON file containing rooms data in the real environment
         * and returns a list of rooms.
         * @param jsonFileName the path of the JSON file
         * @param absolutePath is the absolute path given or not
         */
        std::vector<Door> getEnvDoors(string jsonFileName, bool absolutePath = false);
    };
}

#endif