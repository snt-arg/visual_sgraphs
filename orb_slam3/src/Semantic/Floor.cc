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

    Map *Floor::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Floor::SetMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}