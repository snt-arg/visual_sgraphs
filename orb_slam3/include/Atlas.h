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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "Pinhole.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Semantic/Door.h"
#include "Semantic/Room.h"
#include "Semantic/Floor.h"
#include "KannalaBrandt8.h"
#include "GeometricCamera.h"
#include "Semantic/Marker.h"
#include "Geometric/Plane.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

namespace ORB_SLAM3
{
    class Map;
    class Door;
    class Room;
    class Frame;
    class Plane;
    class Floor;
    class Viewer;
    class Marker;
    class Pinhole;
    class MapPoint;
    class KeyFrame;
    class KannalaBrandt8;
    class KeyFrameDatabase;

    class Atlas
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar.template register_type<Pinhole>();
            ar.template register_type<KannalaBrandt8>();

            // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
            // ar & mspMaps;
            ar & mvpBackupMaps;
            ar & mvpCameras;
            // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
            ar &Map::nNextId;
            ar &Frame::nNextId;
            ar &KeyFrame::nNextId;
            ar &MapPoint::nNextId;
            ar &GeometricCamera::nNextId;
            ar & mnLastInitKFidMap;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Atlas();
        Atlas(int initKFid); // When its initialization the first map is created
        ~Atlas();

        void CreateNewMap();
        void ChangeMap(Map *pMap);

        unsigned long int GetLastInitKFid();

        void SetViewer(Viewer *pViewer);

        // Method for change components in the current map
        void AddMapDoor(Door *door);
        void AddMapRoom(Room *room);
        void AddMapFloor(Floor *floor);
        void AddMapPlane(Plane *plane);
        void AddKeyFrame(KeyFrame *pKF);
        void AddMapPoint(MapPoint *pMP);
        void AddMapMarker(Marker *marker);

        std::vector<GeometricCamera *> GetAllCameras();
        GeometricCamera *AddCamera(GeometricCamera *pCam);

        /* All methods without Map pointer work on current map */
        void InformNewBigChange();
        int GetLastBigChangeIdx();
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        long unsigned MarkersInMap();
        long unsigned KeyFramesInMap();
        long unsigned int MapPointsInMap();

        // List of marker-ids placed on planes detected so far
        std::vector<int> visitedPlanesMarkerIds;

        // Method for get data in current map
        std::vector<Door *> GetAllDoors();
        std::vector<Room *> GetAllRooms();
        std::vector<Floor *> GetAllFloors();
        std::vector<Plane *> GetAllPlanes();
        std::vector<Marker *> GetAllMarkers();
        std::vector<KeyFrame *> GetAllKeyFrames();
        std::vector<MapPoint *> GetAllMapPoints();
        std::vector<MapPoint *> GetReferenceMapPoints();

        Plane *GetBiggestGroundPlane();

        vector<Map *> GetAllMaps();

        int CountMaps();

        void clearMap();

        void clearAtlas();

        Map *GetCurrentMap();

        void SetMapBad(Map *pMap);
        void RemoveBadMaps();

        bool isInertial();
        void SetInertialSensor();
        void SetImuInitialized();
        bool isImuInitialized();

        // Function for garantee the correction of serialization of this object
        void PreSave();
        void PostLoad();

        map<long unsigned int, KeyFrame *> GetAtlasKeyframes();

        // Functions for getting the entities
        Door *GetDoorById(int doorId);
        Room *GetRoomById(int roomId);
        Plane *GetPlaneById(int planeId);
        Floor *GetFloorById(int floorId);
        Marker *GetMarkerById(int markerId);
        KeyFrame *GetKeyFrameById(long unsigned int mnId);

        KeyFrameDatabase *GetKeyFrameDatabase();
        void SetKeyFrameDababase(KeyFrameDatabase *pKFDB);

        ORBVocabulary *GetORBVocabulary();
        void SetORBVocabulary(ORBVocabulary *pORBVoc);

        long unsigned int GetNumLivedKF();
        long unsigned int GetNumLivedMP();

    protected:
        std::set<Map *> mspMaps;
        std::set<Map *> mspBadMaps;
        // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
        std::vector<Map *> mvpBackupMaps;

        Map *mpCurrentMap;

        std::vector<GeometricCamera *> mvpCameras;

        unsigned long int mnLastInitKFidMap;

        Viewer *mpViewer;
        bool mHasViewer;

        // Class references for the map reconstruction from the save file
        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBVocabulary;

        // Mutex
        std::mutex mMutexAtlas;
    };

}

#endif
