/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef MARKER_H
#define MARKER_H

#include "Map.h"
#include "KeyFrame.h"

namespace ORB_SLAM3
{
    class Map;

    class Marker
    {
    private:
        int id;                                           // The marker's identifier
        int opId;                                         // The marker's identifier in the optimizer
        double time;                                      // The timestamp (in seconds) of observing the marker
        bool markerInGMap;                                // Check if the marker is in the Global Map or not
        Sophus::SE3f local_pose;                          // Marker's pose (position and orientation) in the Local Map
        Sophus::SE3f global_pose;                         // Marker's pose (position and orientation) in the Global Map
        std::map<KeyFrame *, Sophus::SE3f> mObservations; // Marker's observations in keyFrames

    public:
        Marker();
        ~Marker();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        double getTime() const;
        void setTime(double value);

        bool isMarkerInGMap() const;
        void setMarkerInGMap(bool value);

        Sophus::SE3f getLocalPose() const;
        void setLocalPose(const Sophus::SE3f &value);

        Sophus::SE3f getGlobalPose() const;
        void setGlobalPose(const Sophus::SE3f &value);

        const std::map<KeyFrame *, Sophus::SE3f> &getObservations() const;
        void addObservation(KeyFrame *pKF);

        Map *GetMap();
        void SetMap(Map *pMap);

    protected:
        Map *mpMap;
        std::mutex mMutexMap;
    };

}

#endif