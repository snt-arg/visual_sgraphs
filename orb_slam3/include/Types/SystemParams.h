
#ifndef SYSTEMPARAMS_H
#define SYSTEMPARAMS_H

namespace ORB_SLAM3
{
    class SystemParams
    {
    public:
        double markerImpact;
        int pointCloudSize_GeoSeg;
        int pointCloudSize_SemSeg;
        double probabilityThreshold_SemSeg;
    };
}

#endif