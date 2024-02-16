
#ifndef SYSTEMPARAMS_H
#define SYSTEMPARAMS_H

namespace ORB_SLAM3
{
    class SystemParams
    {
    public:
        double markerImpact;
        
        // Geometric Segmentation
        int pointCloudSize_GeoSeg;
        
        // Semantic Segmentation
        int pointCloudSize_SemSeg;
        double probabilityThreshold_SemSeg;
    };
}

#endif