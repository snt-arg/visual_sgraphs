/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef MARKER_H
#define MARKER_H

namespace ORB_SLAM3
{
    struct Marker
    {
        // The identifier devoted for each marker
        int id;
        // The timestamp (in seconds) when the marker observed
        double time;
        // The pose (position and orientation) of the marker in the Global Map
        Sophus::SE3f pose;
    };

}

#endif