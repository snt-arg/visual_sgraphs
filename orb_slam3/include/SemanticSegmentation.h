/**
 * 🚀 [vS-Graphs] A Class for Semantic Segmentation of the Scene
 */

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"

namespace ORB_SLAM3
{
    class Atlas;

    class SemanticSegmentation
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SemanticSegmentation(Atlas *pAtlas);

        // Running the thread
        void Run();
    };
}

#endif // SEMANTICSEG_H