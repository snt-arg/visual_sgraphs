/**
 * 🚀 [vS-Graphs] Floor (Storey) Entity
 */

#include "Semantic/Floor.h"

namespace ORB_SLAM3
{
    Floor::Floor() {}
    Floor::~Floor() {}

    int Floor::getId() const
    {
        return id;
    }

    void Floor::setId(int value)
    {
        id = value;
    }

    int Floor::getOpId() const
    {
        return opId;
    }

    void Floor::setOpId(int value)
    {
        opId = value;
    }

    int Floor::getOpIdG() const
    {
        return opIdG;
    }

    void Floor::setOpIdG(int value)
    {
        opIdG = value;
    }

    std::vector<Door *> Floor::getDoors() const
    {
        return doors;
    }

    void Floor::setDoors(Door *value)
    {
        doors.push_back(value);
    }

    std::vector<Room *> Floor::getRooms() const
    {
        return rooms;
    }

    void Floor::setRooms(Room *value)
    {
        rooms.push_back(value);
    }

    std::vector<Plane *> Floor::getWalls() const
    {
        return walls;
    }

    void Floor::setWalls(Plane *value)
    {
        walls.push_back(value);
    }

    Map *Floor::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Floor::setMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}