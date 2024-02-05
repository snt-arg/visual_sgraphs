#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas) {}

    void SemanticSegmentation::Run()
    {
        while (1)
        {
            // Get the unfiltered point cloud of the KF
            // Get the probability map for the pointcloud from Semantic Segmenter
            // For probabilities > 0.9, as the wall, segment all the points in the point cloud
            // Return the pointcloud with only walls and floors
            // Perform distance filter and downsample
            // Send it to the Ransac and get the plane equations
            // Do the association with the planes in Geometric
            usleep(3000);
        }
    }
}