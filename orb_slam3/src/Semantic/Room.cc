/**
 * 🚀 [vS-Graphs] Room (Four-wall Room or Corridor) Entity
 */

#include "Semantic/Room.h"

namespace ORB_SLAM3
{
    Room::Room() {}
    Room::~Room() {}

    int Room::getId() const
    {
        return id;
    }

    void Room::setId(int value)
    {
        id = value;
    }

    int Room::getOpId() const
    {
        return opId;
    }

    void Room::setOpId(int value)
    {
        opId = value;
    }

    int Room::getOpIdG() const
    {
        return opIdG;
    }

    void Room::setOpIdG(int value)
    {
        opIdG = value;
    }

    int Room::getMetaMarkerId() const
    {
        return metaMarkerId;
    }

    void Room::setMetaMarkerId(int value)
    {
        metaMarkerId = value;
    }

    Marker *Room::getMetaMarker() const
    {
        return metaMarker;
    }

    void Room::setMetaMarker(Marker *value)
    {
        metaMarker = value;
    }

    std::string Room::getName() const
    {
        return name;
    }

    void Room::setName(std::string value)
    {
        name = value;
    }

    bool Room::getIsCorridor() const
    {
        return isCorridor;
    }

    void Room::setIsCorridor(bool value)
    {
        isCorridor = value;
    }

    bool Room::getIsCandidate() const
    {
        return isCandidate;
    }

    void Room::setIsCandidate(bool value)
    {
        isCandidate = value;
    }

    std::vector<Plane *> Room::getWalls() const
    {
        return walls;
    }

    void Room::setWalls(Plane *value)
    {
        walls.push_back(value);
    }

    void Room::clearWalls()
    {
        walls.clear();
    }

    std::vector<Door *> Room::getDoors() const
    {
        return doors;
    }

    void Room::setDoors(Door *value)
    {
        doors.push_back(value);
    }

    Eigen::Vector3d Room::getRoomCenter() const
    {
        return roomCenter;
    }

    void Room::setRoomCenter(Eigen::Vector3d value)
    {
        roomCenter = value;
    }

    std::vector<int> Room::getDoorMarkerIds() const
    {
        return doorMarkerIds;
    }

    void Room::setDoorMarkerIds(int value)
    {
        doorMarkerIds.push_back(value);
    }

    std::vector<std::vector<int>> Room::getWallMarkerIds() const
    {
        return wallMarkerIds;
    }

    void Room::setWallMarkerIds(std::vector<int> value)
    {
        wallMarkerIds.push_back(value);
    }

    Map *Room::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Room::SetMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}