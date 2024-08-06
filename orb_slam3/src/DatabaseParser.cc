/**
 * 🚀 [vS-Graphs] Database Parse for JSON Files
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
            std::cout << "- Loading JSON data from " << jsonFilePath << std::endl;
            // Reading the JSON file from the given path
            ifstream jsonFile(jsonFilePath);
            // Parsing the JSON file to get the envrionment data
            json envData = json::parse(jsonFile);
            // Return parsed data
            return envData;
        }
        catch (json::parse_error &ex)
        {
            std::cout << "- Error parsing the environment JSON file: " << ex.what() << std::endl;
            std::cout << "- Exiting ... \n\n";
            exit(1);
        }
    }

    std::vector<Room *> DBParser::getEnvRooms(json envData)
    {
        envRooms.clear();

        // Check if the JSON file contains rooms
        if (envData["rooms"].size() != 0)
        {
            for (const auto &envDatum : envData["rooms"].items())
            {
                // Initialization
                Room *envRoom = new Room();

                // Fill the room entity
                envRoom->setOpId(-1);
                envRoom->setOpIdG(-1);
                envRoom->setId(stoi(envDatum.key()));
                envRoom->setName(envDatum.value()["name"]);
                envRoom->setIsCorridor(envDatum.value()["isCorridor"]);
                envRoom->setMetaMarkerId(envDatum.value()["metaMarker"]);

                // Fill the set of doors (markers attached to doors) of a room
                if (envDatum.value()["doorMarkers"].size() != 0)
                    for (const auto &marker : envDatum.value()["doorMarkers"].items())
                        envRoom->setDoorMarkerIds(marker.value());
                else
                    std::cout << "- No doors connected to the room '" << envRoom->getName() << "'!" << std::endl;

                // Fill the vector
                envRooms.push_back(envRoom);
            }

            // Print the loaded rooms
            std::cout << "- Fetched " << envRooms.size() << " rooms from the JSON file! [e.g., '"
                      << envRooms[0]->getName() << "']." << std::endl;
        }
        else
            std::cout << "- No rooms found in the JSON file!" << std::endl;

        return envRooms;
    }

    std::vector<Door *> DBParser::getEnvDoors(json envData)
    {
        envDoors.clear();

        // Check if the JSON file contains doors
        if (envData["doors"].size() != 0)
        {
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
        }
        else
            std::cout << "- No doors found in the JSON file!" << std::endl;

        return envDoors;
    }
}