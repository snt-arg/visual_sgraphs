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

#include "Atlas.h"
#include "Viewer.h"

#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "GeometricCamera.h"

namespace VS_GRAPHS
{
    Atlas::Atlas()
    {
        mpCurrentMap = static_cast<Map *>(NULL);
    }

    Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid), mHasViewer(false)
    {
        mpCurrentMap = static_cast<Map *>(NULL);
        CreateNewMap();
    }

    Atlas::~Atlas()
    {
        for (std::set<Map *>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
        {
            Map *pMi = *it;

            if (pMi)
            {
                delete pMi;
                pMi = static_cast<Map *>(NULL);

                it = mspMaps.erase(it);
            }
            else
                ++it;
        }
    }

    void Atlas::CreateNewMap()
    {
        // Lock the map creation
        unique_lock<mutex> lock(mMutexAtlas);
        std::cout << "\n[Atlas]" << std::endl;
        std::cout << "- Creating a new map (MapId: " << Map::nNextId << ", Init KeyFrame: "
                  << mnLastInitKFidMap << ") ..." << std::endl;

        if (mpCurrentMap)
        {
            if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
                mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;

            mpCurrentMap->SetStoredMap();
            std::cout << "- The created map with MapId #" << mpCurrentMap->GetId() << " has been stored!" << std::endl;
        }

        mpCurrentMap = new Map(mnLastInitKFidMap);
        mpCurrentMap->SetCurrentMap();
        mspMaps.insert(mpCurrentMap);
    }

    void Atlas::ChangeMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        std::cout << "\n[Atlas]" << std::endl;
        std::cout << "- Changing to map with MapId #" << pMap->GetId() << " ..." << std::endl;

        if (mpCurrentMap)
            mpCurrentMap->SetStoredMap();

        mpCurrentMap = pMap;
        mpCurrentMap->SetCurrentMap();
    }

    unsigned long int Atlas::GetLastInitKFid()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mnLastInitKFidMap;
    }

    void Atlas::SetViewer(Viewer *pViewer)
    {
        mpViewer = pViewer;
        mHasViewer = true;
    }

    void Atlas::AddKeyFrame(KeyFrame *pKF)
    {
        Map *pMapKF = pKF->GetMap();
        pMapKF->AddKeyFrame(pKF);
    }

    void Atlas::AddMapPoint(MapPoint *pMP)
    {
        Map *pMapMP = pMP->GetMap();
        pMapMP->AddMapPoint(pMP);
    }

    void Atlas::AddMapMarker(Marker *marker)
    {
        Map *pMapMP = marker->getMap();
        pMapMP->AddMapMarker(marker);
    }

    void Atlas::AddMapPlane(Plane *plane)
    {
        // Add it to the map
        Map *pMapMP = plane->GetMap();
        pMapMP->AddMapPlane(plane);
    }

    void Atlas::AddMapDoor(Door *door)
    {
        Map *pMapMP = door->getMap();
        pMapMP->AddMapDoor(door);
    }

    void Atlas::AddDetectedMapRoom(Room *room)
    {
        Map *pMapMP = room->getMap();
        pMapMP->AddDetectedMapRoom(room);
    }

    void Atlas::AddMarkerBasedMapRoom(Room *room)
    {
        Map *pMapMP = room->getMap();
        pMapMP->AddMarkerBasedMapRoom(room);
    }

    void Atlas::AddMapFloor(Floor *floor)
    {
        Map *pMapMP = floor->getMap();
        pMapMP->AddMapFloor(floor);
    }

    GeometricCamera *Atlas::AddCamera(GeometricCamera *pCam)
    {
        // Check if the camera already exists
        bool bAlreadyInMap = false;
        int index_cam = -1;
        for (size_t i = 0; i < mvpCameras.size(); ++i)
        {
            GeometricCamera *pCam_i = mvpCameras[i];
            if (!pCam)
                std::cout << "Not pCam" << std::endl;
            if (!pCam_i)
                std::cout << "Not pCam_i" << std::endl;
            if (pCam->GetType() != pCam_i->GetType())
                continue;

            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
            {
                if (((Pinhole *)pCam_i)->IsEqual(pCam))
                {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            }
            else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE)
            {
                if (((KannalaBrandt8 *)pCam_i)->IsEqual(pCam))
                {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            }
        }

        if (bAlreadyInMap)
        {
            return mvpCameras[index_cam];
        }
        else
        {
            mvpCameras.push_back(pCam);
            return pCam;
        }
    }

    std::vector<GeometricCamera *> Atlas::GetAllCameras()
    {
        return mvpCameras;
    }

    void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetReferenceMapPoints(vpMPs);
    }

    void Atlas::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->InformNewBigChange();
    }

    int Atlas::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetLastBigChangeIdx();
    }

    long unsigned int Atlas::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->MapPointsInMap();
    }

    long unsigned int Atlas::MarkersInMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->MarkersInMap();
    }

    long unsigned Atlas::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->KeyFramesInMap();
    }

    std::vector<std::vector<Eigen::Vector3d>> Atlas::GetSkeletoClusterPoints()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetSkeletoClusterPoints();
    }

    void Atlas::SetSkeletonClusterPoints(const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetSkeletonClusterPoints(newClusterPoints);
    }

    std::vector<KeyFrame *> Atlas::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllKeyFrames();
    }

    KeyFrame *Atlas::GetKeyFrameById(long unsigned int mnId)
    {
        KeyFrame *retrievedKF = mpCurrentMap->GetKeyFrameById(mnId);
        return retrievedKF;
    }

    Door *Atlas::GetDoorById(int doorId)
    {
        Door *fetchedDoor = mpCurrentMap->GetDoorById(doorId);
        return fetchedDoor;
    }

    Floor *Atlas::GetFloorById(int floorId)
    {
        Floor *fetchedFloor = mpCurrentMap->GetFloorById(floorId);
        return fetchedFloor;
    }

    Plane *Atlas::GetPlaneById(int planeId)
    {
        Plane *fetchedPlane = mpCurrentMap->GetPlaneById(planeId);
        return fetchedPlane;
    }

    Marker *Atlas::GetMarkerById(int markerId)
    {
        Marker *fetchedMarker = mpCurrentMap->GetMarkerById(markerId);
        return fetchedMarker;
    }

    std::vector<MapPoint *> Atlas::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllMapPoints();
    }

    std::vector<Marker *> Atlas::GetAllMarkers()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllMarkers();
    }

    std::vector<Plane *> Atlas::GetAllPlanes()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllPlanes();
    }

    std::vector<Door *> Atlas::GetAllDoors()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllDoors();
    }

    std::vector<Room *> Atlas::GetAllRooms()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllRooms();
    }

    std::vector<Room *> Atlas::GetAllDetectedMapRooms()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllDetectedMapRooms();
    }

    std::vector<Room *> Atlas::GetAllMarkerBasedMapRooms()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllMarkerBasedMapRooms();
    }

    std::vector<Floor *> Atlas::GetAllFloors()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllFloors();
    }

    Plane *Atlas::GetBiggestGroundPlane()
    {
        return mpCurrentMap->GetBiggestGroundPlane();
    }

    std::vector<MapPoint *> Atlas::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetReferenceMapPoints();
    }

    vector<Map *> Atlas::GetAllMaps()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        struct compFunctor
        {
            inline bool operator()(Map *elem1, Map *elem2)
            {
                return elem1->GetId() < elem2->GetId();
            }
        };
        vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
        sort(vMaps.begin(), vMaps.end(), compFunctor());
        return vMaps;
    }

    int Atlas::CountMaps()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mspMaps.size();
    }

    void Atlas::clearMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->clear();
    }

    void Atlas::clearAtlas()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mspMaps.clear();
        mpCurrentMap = static_cast<Map *>(NULL);
        mnLastInitKFidMap = 0;
    }

    Map *Atlas::GetCurrentMap()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        if (!mpCurrentMap)
            CreateNewMap();
        while (mpCurrentMap->IsBad())
            usleep(3000);

        return mpCurrentMap;
    }

    void Atlas::SetMapBad(Map *pMap)
    {
        mspMaps.erase(pMap);
        pMap->SetBad();

        mspBadMaps.insert(pMap);
    }

    void Atlas::RemoveBadMaps()
    {
        mspBadMaps.clear();
    }

    bool Atlas::isInertial()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->IsInertial();
    }

    void Atlas::SetInertialSensor()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetInertialSensor();
    }

    void Atlas::SetImuInitialized()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetImuInitialized();
    }

    bool Atlas::isImuInitialized()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->isImuInitialized();
    }

    void Atlas::PreSave()
    {
        if (mpCurrentMap)
        {
            if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
                mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1; // The init KF is the next of current maximum
        }

        struct compFunctor
        {
            inline bool operator()(Map *elem1, Map *elem2)
            {
                return elem1->GetId() < elem2->GetId();
            }
        };
        std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
        sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

        std::set<GeometricCamera *> spCams(mvpCameras.begin(), mvpCameras.end());
        for (Map *pMi : mvpBackupMaps)
        {
            if (!pMi || pMi->IsBad())
                continue;

            if (pMi->GetAllKeyFrames().size() == 0)
            {
                // Empty map, erase before of save it.
                SetMapBad(pMi);
                continue;
            }
            pMi->PreSave(spCams);
        }
        RemoveBadMaps();
    }

    void Atlas::PostLoad()
    {
        map<unsigned int, GeometricCamera *> mpCams;
        for (GeometricCamera *pCam : mvpCameras)
        {
            mpCams[pCam->GetId()] = pCam;
        }

        mspMaps.clear();
        unsigned long int numKF = 0, numMP = 0;
        for (Map *pMi : mvpBackupMaps)
        {
            mspMaps.insert(pMi);
            pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
            numKF += pMi->GetAllKeyFrames().size();
            numMP += pMi->GetAllMapPoints().size();
        }
        mvpBackupMaps.clear();
    }

    void Atlas::SetKeyFrameDababase(KeyFrameDatabase *pKFDB)
    {
        mpKeyFrameDB = pKFDB;
    }

    KeyFrameDatabase *Atlas::GetKeyFrameDatabase()
    {
        return mpKeyFrameDB;
    }

    void Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        mpORBVocabulary = pORBVoc;
    }

    ORBVocabulary *Atlas::GetORBVocabulary()
    {
        return mpORBVocabulary;
    }

    long unsigned int Atlas::GetNumLivedKF()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        long unsigned int num = 0;
        for (Map *pMap_i : mspMaps)
        {
            num += pMap_i->GetAllKeyFrames().size();
        }

        return num;
    }

    long unsigned int Atlas::GetNumLivedMP()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        long unsigned int num = 0;
        for (Map *pMap_i : mspMaps)
        {
            num += pMap_i->GetAllMapPoints().size();
        }

        return num;
    }

    map<long unsigned int, KeyFrame *> Atlas::GetAtlasKeyframes()
    {
        map<long unsigned int, KeyFrame *> mpIdKFs;
        for (Map *pMap_i : mvpBackupMaps)
        {
            vector<KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

            for (KeyFrame *pKF_j_Mi : vpKFs_Mi)
            {
                mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
            }
        }

        return mpIdKFs;
    }

}
