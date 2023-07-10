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
    // [TODO]: make all the variables in the class private and access them using proper add and get functions
    class Marker
    {
    public:
        // The identifier devoted for each marker
        int id;
        // The identifier devoted for the optimizer
        int mId;
        // The timestamp (in seconds) when the marker observed
        double time;
        // The boolean to check if the marker is in the Global Map or not
        bool markerInMap;
        // The pose (position and orientation) of the marker in the Local Map
        Sophus::SE3f local_pose;
        // The pose (position and orientation) of the marker in the Global Map
        Sophus::SE3f global_pose;
        // The observations of the markers
        std::map<KeyFrame *, Sophus::SE3f> mObservations;

    public:
        void AddObservation(KeyFrame *pKF)
        {
            mObservations.insert({pKF, local_pose});
        }
    };

}

#endif