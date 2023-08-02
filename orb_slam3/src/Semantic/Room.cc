/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
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

    std::string Room::getName() const
    {
        return name;
    }

    void Room::setName(std::string value)
    {
        name = value;
    }

    bool Room::getAllMarkersSeen() const
    {
        return all_markers_seen;
    }

    void Room::setAllMarkersSeen(bool value)
    {
        all_markers_seen = value;
    }

    std::vector<Wall *> Room::getWalls() const
    {
        return walls;
    }

    void Room::setWalls(std::vector<Wall *> &value)
    {
        walls = value;
    }

    Eigen::Vector3d Room::getRoomCenter() const
    {
        return room_center;
    }

    void Room::setRoomCenter(Eigen::Vector3d value)
    {
        room_center = value;
    }

    std::vector<std::vector<int>> Room::getMarkerIds() const
    {
        return marker_ids;
    }

    void Room::setMarkerIds(std::vector<std::vector<int>> value)
    {
        marker_ids = value;
    }
}