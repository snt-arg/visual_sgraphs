#include "GeometricSegmentation.h"

namespace ORB_SLAM3
{
    GeometricSegmentation::GeometricSegmentation(Atlas *pAtlas) {}

    void GeometricSegmentation::Run()
    {
        while (1)
        {
            // Check if there are new keyframes in the buffer
            // Apply RANSAC
            // Add the new planes to the KF
            usleep(3000);
        }
    }
}