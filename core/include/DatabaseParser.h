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

#ifndef DBPARSER_H
#define DBPARSER_H

#include <fstream>
#include <iostream>
#include "Thirdparty/nlohmann/json.hpp"

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
        std::vector<Room *> envRooms; // Rooms available in the real environment

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
         * @param envData the JSON file containing the rooms data
         */
        std::vector<Room *> getEnvRooms(json envData);
    };
}

#endif