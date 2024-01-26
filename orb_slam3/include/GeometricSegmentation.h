/**
 * 🚀 [vS-Graphs] A Class for Geometric Segmentation of the Scene
 */

#ifndef GEOMETRICSEG_H
#define GEOMETRICSEG_H

#include "Atlas.h"

namespace ORB_SLAM3
{
    class Atlas;

    class GeometricSegmentation
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GeometricSegmentation(Atlas *pAtlas);

        // Running the thread
        void Run();
    };
}

#endif // GEOMETRICSEG_H