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
            std::cout << "\nLoading JSON data from '" << jsonFilePath << "'" << std::endl;
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

    std::vector<Room *> DBParser::getEnvRooms(json envData)
    {
        envRooms.clear();

        // Iterate over all rooms data in JSON
        for (const auto &envDatum : envData["rooms"].items())
        {
            // Initialization
            Room *envRoom = new Room();

            // Fill the room entity
            envRoom->setOpId(-1);
            envRoom->setOpIdG(-1);
            envRoom->setAllSeenMarkers(false);
            envRoom->setId(stoi(envDatum.key()));
            envRoom->setName(envDatum.value()["name"]);

            // Fill the set of walls (marker-pairs attached to walls) of a room
            for (int idx = 0; idx < envDatum.value()["markers"].size(); idx++)
            {
                // Get the marker IDs of a wall
                std::vector<int> markerIds;
                for (const auto &marker : envDatum.value()["markers"][idx].items())
                    markerIds.push_back(marker.value());
                // Add the marker IDs to the set of walls
                envRoom->setWallMarkerIds(markerIds);
            }

            // Fill the set of doors (markers attached to doors) of a room
            for (const auto &marker : envDatum.value()["doorMarkers"].items())
                envRoom->setDoorMarkerIds(marker.value());

            // Fill the vector
            envRooms.push_back(envRoom);
        }

        // Print the loaded rooms
        std::cout << "- Fetched " << envRooms.size() << " rooms from the JSON file! [e.g., '"
                  << envRooms[0]->getName() << "']." << std::endl;

        return envRooms;
    }

    std::vector<Door *> DBParser::getEnvDoors(json envData)
    {
        envDoors.clear();

        // Iterate over all rooms data in JSON
        for (const auto &envDatum : envData["doors"].items())
        {
            // Initialization
            Door *envDoor = new Door();

            // Fill the room entity
            envDoor->setOpId(-1);
            envDoor->setOpIdG(-1);
            envDoor->setId(stoi(envDatum.key()));
            envDoor->setName(envDatum.value()["name"]);
            envDoor->setMarkerId(envDatum.value()["marker"]);

            // Fill the vector
            envDoors.push_back(envDoor);
        }

        // Print the loaded doors
        std::cout << "- Fetched " << envDoors.size() << " doors from the JSON file! [e.g., '"
                  << envDoors[0]->getName() << "']." << std::endl;

        return envDoors;
    }
}