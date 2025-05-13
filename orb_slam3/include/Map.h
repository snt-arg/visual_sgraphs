/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Semantic/Room.h"
#include "Semantic/Floor.h"
#include "Semantic/Marker.h"

#include <set>
#include <mutex>
#include <unordered_map>
#include <pangolin/pangolin.h>
#include <boost/serialization/base_object.hpp>

namespace VS_GRAPHS
{
    class Door;
    class Room;
    class Atlas;
    class Plane;
    class Floor;
    class Marker;
    class MapPoint;
    class KeyFrame;
    class KeyFrameDatabase;

    class Map
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & mnId;
            ar & mnInitKFid;
            ar & mnMaxKFid;
            ar & mnBigChangeIdx;

            // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
            // ar & mspKeyFrames;
            // ar & mspMapPoints;
            ar & mvpBackupKeyFrames;
            ar & mvpBackupMapPoints;

            ar & mvBackupKeyFrameOriginsId;

            ar & mnBackupKFinitialID;
            ar & mnBackupKFlowerID;

            ar & mbImuInitialized;
            ar & mbIsInertial;
            ar & mbIMU_BA1;
            ar & mbIMU_BA2;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Map();
        Map(int initKFid);
        ~Map();

        void AddMapDoor(Door *pDoor);
        void AddMapFloor(Floor *pFloor);
        void AddKeyFrame(KeyFrame *pKF);
        void AddMapPoint(MapPoint *pMP);
        void AddMapPlane(Plane *pPlane);
        void AddMapMarker(Marker *pMarker);
        void AddDetectedMapRoom(Room *room);
        void AddMarkerBasedMapRoom(Room *room);

        void EraseMapDoor(Door *pDoor);
        void EraseMapFloor(Floor *pFloor);
        void EraseMapPoint(MapPoint *pMP);
        void EraseKeyFrame(KeyFrame *pKF);
        void EraseMapPlane(Plane *pPlane);
        void EraseMapMarker(Marker *pMarker);
        void EraseDetectedMapRoom(Room *pRoom);
        void EraseMarkerBasedMapRoom(Room *pRoom);

        void InformNewBigChange();
        int GetLastBigChangeIdx();
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        std::vector<Door *> GetAllDoors();
        std::vector<Room *> GetAllRooms();
        std::vector<Floor *> GetAllFloors();
        std::vector<Plane *> GetAllPlanes();
        std::vector<Marker *> GetAllMarkers();
        std::vector<KeyFrame *> GetAllKeyFrames();
        std::vector<MapPoint *> GetAllMapPoints();
        std::vector<Room *> GetAllDetectedMapRooms();
        std::vector<Room *> GetAllMarkerBasedMapRooms();
        std::vector<MapPoint *> GetReferenceMapPoints();

        /**
         * @brief Get the cluster points of the map set by `voxblox_skeleton`
         */
        std::vector<std::vector<Eigen::Vector3d>> GetSkeletoClusterPoints();

        /**
         * @brief Set the cluster points of the map set by `voxblox_skeleton`
         * @param newClusterPoints The new cluster points to set
         */
        void SetSkeletonClusterPoints(const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints);

        long unsigned KeyFramesInMap();
        long unsigned int MarkersInMap();
        long unsigned int MapPointsInMap();

        long unsigned int GetId();
        long unsigned int GetMaxKFid();
        long unsigned int GetInitKFid();
        void SetInitKFid(long unsigned int initKFif);

        KeyFrame *GetOriginKF();
        Door *GetDoorById(int doorId);
        Floor *GetFloorById(int floorId);
        Plane *GetPlaneById(int planeId);
        Marker *GetMarkerById(int markerId);
        KeyFrame *GetKeyFrameById(long unsigned int mnId);

        Plane *GetBiggestGroundPlane();

        void SetStoredMap();
        void SetCurrentMap();

        bool IsInUse();

        bool IsBad();
        void SetBad();

        void clear();

        int GetLastMapChange();
        int GetMapChangeIndex();
        void IncreaseChangeIndex();
        void SetLastMapChange(int currentChangeId);

        bool isImuInitialized();
        void SetImuInitialized();

        void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel = false);

        bool IsInertial();
        void SetIniertialBA1();
        void SetIniertialBA2();
        bool GetIniertialBA1();
        bool GetIniertialBA2();
        void SetInertialSensor();

        void ChangeId(long unsigned int nId);

        unsigned int GetLowerKFID();

        void PreSave(std::set<GeometricCamera *> &spCams);
        void PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc, map<unsigned int, GeometricCamera *> &mpCams);

        KeyFrame *mpFirstRegionKF;
        std::mutex mMutexMapUpdate;
        vector<KeyFrame *> mvpKeyFrameOrigins;
        vector<unsigned long int> mvBackupKeyFrameOriginsId;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

        bool mbFail;

        // Size of the thumbnail (always in power of 2)
        static const int THUMB_WIDTH = 512;
        static const int THUMB_HEIGHT = 512;

        static long unsigned int nNextId;

        std::set<long unsigned int> msOptKFs;
        std::set<long unsigned int> msFixedKFs;

    protected:
        long unsigned int mnId;

        std::set<Door *> mspDoors;
        std::set<Floor *> mspFloors;
        std::set<Plane *> mspPlanes;
        std::set<Marker *> mspMarkers;
        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *> mspKeyFrames;
        std::set<Room *> mspDetectedRooms;
        std::set<Room *> mspMarkerBasedRooms;

        // Skeleton cluster points of the map set by `voxblox_skeleton`
        std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

        // Hashmaps and indices for fetching elements
        std::unordered_map<int, Door *> mDoorIndex;
        std::unordered_map<int, Floor *> mFloorIndex;
        std::unordered_map<int, Plane *> mPlaneIndex;
        std::unordered_map<int, Marker *> mMarkerIndex;
        std::unordered_map<long unsigned int, KeyFrame *> mKFIndex;

        // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        std::vector<MapPoint *> mvpBackupMapPoints;
        std::vector<KeyFrame *> mvpBackupKeyFrames;

        KeyFrame *mpKFinitial;
        KeyFrame *mpKFlowerID;

        unsigned long int mnBackupKFlowerID;
        unsigned long int mnBackupKFinitialID;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        bool mbImuInitialized;

        int mnMapChange;
        int mnMapChangeNotified;

        long unsigned int mnInitKFid;
        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        // View of the map in aerial sight (for the AtlasViewer)
        GLubyte *mThumbnail;

        bool mIsInUse;
        bool mHasTumbnail;
        bool mbBad = false;

        bool mbIsInertial;
        bool mbIMU_BA1;
        bool mbIMU_BA2;

        // Mutex
        std::mutex mMutexMap;
    };

}

#endif
