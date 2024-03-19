/**
 * 🚀 [vS-Graphs] Floor (Storey) Entity
 */

#ifndef FLOOR_H
#define FLOOR_H

#include "Map.h"

namespace ORB_SLAM3
{

    class Floor
    {
    private:
        int id;    // The floor's identifier
        int opId;  // The floor's identifier in the local optimizer
        int opIdG; // The floor's identifier in the global optimizer

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Floor();
        ~Floor();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        Map *GetMap();
        void SetMap(Map *pMap);

    protected:
        Map *mpMap;
        std::mutex mMutexMap;
    };
}

#endif