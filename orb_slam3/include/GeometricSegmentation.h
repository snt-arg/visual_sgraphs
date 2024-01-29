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
    private:
        Atlas *mpAtlas;
        std::mutex mMutexNewKFs;
        std::list<KeyFrame *> mvpKeyFrameBuffer;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GeometricSegmentation(Atlas *pAtlas);

        void AddKeyFrameToBuffer(KeyFrame *pKF);
        std::list<KeyFrame *> GetKeyFrameBuffer();

        // Running the thread
        void Run();
    };
}

#endif // GEOMETRICSEG_H