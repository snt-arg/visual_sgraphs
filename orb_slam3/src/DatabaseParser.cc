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
        std::cout << "\nFetching rooms data ...";

        // Iterate over all rooms data in JSON
        for (const auto &envDatum : envData["rooms"].items())
        {
            // Initialization
            Room envRoom;
            std::string roomMarkersToShow("");
            std::vector<std::vector<int>> roomMarkerIds;

            // Fetch each marker from the vector as set of marker-pairs attached to a room
            for (int idx = 0; idx < envDatum.value()["markers"].size(); idx++)
            {
                std::vector<int> readMarkerIds;
                for (const auto &marker : envDatum.value()["markers"][idx].items())
                {
                    readMarkerIds.push_back(marker.value());
                    roomMarkersToShow.append(to_string(marker.value()) + " ");
                }
                roomMarkerIds.push_back(readMarkerIds);
            }
            std::cout << "- Room#" << envDatum.key() << " (" << envDatum.value()["name"]
                      << ") fetched with markers [ " << roomMarkersToShow << "]." << std::endl;

            // Fill the room entity
            envRoom.setOpId(-1);
            envRoom.setOpIdG(-1);
            envRoom.setAllMarkersSeen(false);
            envRoom.setId(stoi(envDatum.key()));
            envRoom.setMarkerIds(roomMarkerIds);
            envRoom.setName(envDatum.value()["name"]);

            // Fill the vector
            envRooms.push_back(envRoom);
        }

        return envRooms;
    }

    std::vector<Door> DBParser::getEnvDoors(json envData)
    {
        envDoors.clear();
        std::cout << "\nFetching doors data from JSON ...";

        // Iterate over all rooms data in JSON
        for (const auto &envDatum : envData["doors"].items())
        {
            // Initialization
            Door envDoor;
            Marker *envMarker = new Marker();

            std::cout << "- Door#" << envDatum.key() << " (" << envDatum.value()["name"]
                      << ") fetched with marker " << envDatum.value()["marker"] << "." << std::endl;

            // Fill the attached marker
            envMarker->setTime(0);
            envMarker->setOpId(-1);
            envMarker->setOpIdG(-1);
            envMarker->setMarkerInGMap(false);
            envMarker->setId(envDatum.value()["marker"]);

            // Fill the room entity
            envDoor.setOpId(-1);
            envDoor.setOpIdG(-1);
            envDoor.setMarker(envMarker);
            envDoor.setId(stoi(envDatum.key()));
            envDoor.setName(envDatum.value()["name"]);

            // Fill the vector
            envDoors.push_back(envDoor);

            return envDoors;
        }
    }
}