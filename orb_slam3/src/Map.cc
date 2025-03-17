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

#include "Map.h"

#include <mutex>

namespace VS_GRAPHS
{

    long unsigned int Map::nNextId = 0;

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
                 mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
    {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(NULL);
    }

    Map::Map(int initKFid) : mnInitKFid(initKFid), mnMaxKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
                             mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
                             mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
    {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(NULL);
    }

    Map::~Map()
    {
        // TODO: erase all points from memory
        mspMapPoints.clear();

        // TODO: erase all keyframes from memory
        mspKeyFrames.clear();

        // Erase all markers from memory
        mspMarkers.clear();

        // Erase all semantic entities from memory
        mspDoors.clear();
        mspFloors.clear();
        mspPlanes.clear();
        mspDetectedRooms.clear();
        mspMarkerBasedRooms.clear();

        if (mThumbnail)
            delete mThumbnail;
        mThumbnail = static_cast<GLubyte *>(NULL);

        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);

        // Check if the KeyFrames are already in the map
        if (mspKeyFrames.empty())
        {
            std::cout << "\n[Mapping]" << std::endl;
            std::cout << "- Map initialized with initial KeyFrame #" << mnInitKFid << std::endl;
            mnInitKFid = pKF->mnId;
            mpKFinitial = pKF;
            mpKFlowerID = pKF;
        }

        // Add the KeyFrame to the map
        mspKeyFrames.insert(pKF);

        // Update the maximum KeyFrame id
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;

        if (pKF->mnId < mpKFlowerID->mnId)
            mpKFlowerID = pKF;

        mKFIndex[pKF->mnId] = pKF;
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::AddMapMarker(Marker *pMarker)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMarkers.insert(pMarker);
        // Add the marker to the hashmap
        mMarkerIndex[pMarker->getId()] = pMarker;
    }

    void Map::AddMapPlane(Plane *pPlane)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspPlanes.insert(pPlane);
        // Add the plane to the hashmap
        mPlaneIndex[pPlane->getId()] = pPlane;
    }

    void Map::AddMapDoor(Door *pDoor)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspDoors.insert(pDoor);
        // Add the door to the hashmap
        mDoorIndex[pDoor->getId()] = pDoor;
    }

    void Map::AddDetectedMapRoom(Room *pRoom)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspDetectedRooms.insert(pRoom);
    }

    void Map::AddMarkerBasedMapRoom(Room *pRoom)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMarkerBasedRooms.insert(pRoom);
    }

    void Map::AddMapFloor(Floor *pFloor)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspFloors.insert(pFloor);
        // Add the floor to the hashmap
        mFloorIndex[pFloor->getId()] = pFloor;
    }

    KeyFrame *Map::GetKeyFrameById(long unsigned int mnId)
    {
        unique_lock<mutex> lock(mMutexMap);
        KeyFrame *retrievedKF = mKFIndex[mnId];
        return retrievedKF;
    }

    Door *Map::GetDoorById(int doorId)
    {
        unique_lock<mutex> lock(mMutexMap);
        Door *fetchedDoor = mDoorIndex[doorId];
        return fetchedDoor;
    }

    Plane *Map::GetPlaneById(int planeId)
    {
        unique_lock<mutex> lock(mMutexMap);
        Plane *fetchedPlane = mPlaneIndex[planeId];
        return fetchedPlane;
    }

    Marker *Map::GetMarkerById(int markerId)
    {
        unique_lock<mutex> lock(mMutexMap);
        Marker *fetchedMarker = mMarkerIndex[markerId];
        return fetchedMarker;
    }

    Floor *Map::GetFloorById(int floorId)
    {
        unique_lock<mutex> lock(mMutexMap);
        Floor *fetchedFloor = mFloorIndex[floorId];
        return fetchedFloor;
    }

    void Map::SetImuInitialized()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbImuInitialized = true;
    }

    bool Map::isImuInitialized()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbImuInitialized;
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseMapMarker(Marker *pMarker)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMarkers.erase(pMarker);
    }

    void Map::EraseMapPlane(Plane *pPlane)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspPlanes.erase(pPlane);
    }

    void Map::EraseMapDoor(Door *pDoor)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspDoors.erase(pDoor);
    }

    void Map::EraseDetectedMapRoom(Room *pRoom)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspDetectedRooms.erase(pRoom);
    }

    void Map::EraseMarkerBasedMapRoom(Room *pRoom)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMarkerBasedRooms.erase(pRoom);
    }

    void Map::EraseMapFloor(Floor *pFloor)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspFloors.erase(pFloor);
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);
        if (mspKeyFrames.size() > 0)
        {
            if (pKF->mnId == mpKFlowerID->mnId)
            {
                vector<KeyFrame *> vpKFs = vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
                sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
                mpKFlowerID = vpKFs[0];
            }
        }
        else
        {
            mpKFlowerID = 0;
        }

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    std::vector<std::vector<Eigen::Vector3d>> Map::GetSkeletoClusterPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return skeletonClusterPoints;
    }

    void Map::SetSkeletonClusterPoints(const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints)
    {
        unique_lock<mutex> lock(mMutexMap);
        skeletonClusterPoints = newClusterPoints;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    vector<Marker *> Map::GetAllMarkers()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Marker *>(mspMarkers.begin(), mspMarkers.end());
    }

    vector<Plane *> Map::GetAllPlanes()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Plane *>(mspPlanes.begin(), mspPlanes.end());
    }

    Plane *Map::GetBiggestGroundPlane()
    {
        Plane *biggestGroundPlane = nullptr;
        size_t maxPoints = 0;
        vector<Plane *> planes = GetAllPlanes();
        for (auto sit = planes.begin(); sit != planes.end(); sit++)
        {
            Plane *pPlane = *sit;
            if (pPlane->getPlaneType() == Plane::planeVariant::GROUND)
            {
                size_t numPoints = pPlane->getMapClouds()->size();
                if (numPoints > maxPoints)
                {
                    maxPoints = numPoints;
                    biggestGroundPlane = pPlane;
                }
            }
        }
        return biggestGroundPlane;
    }

    vector<Door *> Map::GetAllDoors()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Door *>(mspDoors.begin(), mspDoors.end());
    }

    vector<Room *> Map::GetAllRooms()
    {
        unique_lock<mutex> lock(mMutexMap);
        vector<Room *> allRooms;
        allRooms.insert(allRooms.end(), mspDetectedRooms.begin(), mspDetectedRooms.end());
        allRooms.insert(allRooms.end(), mspMarkerBasedRooms.begin(), mspMarkerBasedRooms.end());
        return allRooms;
    }

    vector<Room *> Map::GetAllDetectedMapRooms()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Room *>(mspDetectedRooms.begin(), mspDetectedRooms.end());
    }

    vector<Room *> Map::GetAllMarkerBasedMapRooms()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Room *>(mspMarkerBasedRooms.begin(), mspMarkerBasedRooms.end());
    }

    vector<Floor *> Map::GetAllFloors()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Floor *>(mspFloors.begin(), mspFloors.end());
    }

    long unsigned int Map::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::MarkersInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMarkers.size();
    }

    long unsigned int Map::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetId()
    {
        return mnId;
    }

    long unsigned int Map::GetInitKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnInitKFid;
    }

    void Map::SetInitKFid(long unsigned int initKFif)
    {
        unique_lock<mutex> lock(mMutexMap);
        mnInitKFid = initKFif;
    }

    long unsigned int Map::GetMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    KeyFrame *Map::GetOriginKF()
    {
        return mpKFinitial;
    }

    void Map::SetCurrentMap()
    {
        mIsInUse = true;
    }

    void Map::SetStoredMap()
    {
        mIsInUse = false;
    }

    void Map::clear()
    {
        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
        {
            KeyFrame *pKF = *sit;
            pKF->UpdateMap(static_cast<Map *>(NULL));
        }

        mspDoors.clear();
        mspPlanes.clear();
        mspMarkers.clear();
        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = mnInitKFid;
        mbImuInitialized = false;
        mspDetectedRooms.clear();
        mspMarkerBasedRooms.clear();
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
        mbIMU_BA1 = false;
        mbIMU_BA2 = false;
    }

    bool Map::IsInUse()
    {
        return mIsInUse;
    }

    void Map::SetBad()
    {
        mbBad = true;
    }

    bool Map::IsBad()
    {
        return mbBad;
    }

    void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
    {
        unique_lock<mutex> lock(mMutexMap);

        // Body position (IMU) of first keyframe is fixed to (0,0,0)
        Sophus::SE3f Tyw = T;
        Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
        Eigen::Vector3f tyw = Tyw.translation();

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++)
        {
            KeyFrame *pKF = *sit;
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Twc.translation() *= s;
            Sophus::SE3f Tyc = Tyw * Twc;
            Sophus::SE3f Tcy = Tyc.inverse();
            pKF->SetPose(Tcy);
            Eigen::Vector3f Vw = pKF->GetVelocity();
            if (!bScaledVel)
                pKF->SetVelocity(Ryw * Vw);
            else
                pKF->SetVelocity(Ryw * Vw * s);
        }

        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++)
        {
            MapPoint *pMP = *sit;
            pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
            pMP->UpdateNormalAndDepth();
        }

        for (set<Marker *>::iterator sit = mspMarkers.begin(); sit != mspMarkers.end(); sit++)
        {
            // MapPoint *pMP = *sit; [TODO]
            // pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
            // pMP->UpdateNormalAndDepth();
        }
        mnMapChange++;
    }

    void Map::SetInertialSensor()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbIsInertial = true;
    }

    bool Map::IsInertial()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbIsInertial;
    }

    void Map::SetIniertialBA1()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA1 = true;
    }

    void Map::SetIniertialBA2()
    {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA2 = true;
    }

    bool Map::GetIniertialBA1()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA1;
    }

    bool Map::GetIniertialBA2()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA2;
    }

    void Map::ChangeId(long unsigned int nId)
    {
        mnId = nId;
    }

    unsigned int Map::GetLowerKFID()
    {
        unique_lock<mutex> lock(mMutexMap);
        if (mpKFlowerID)
        {
            return mpKFlowerID->mnId;
        }
        return 0;
    }

    int Map::GetMapChangeIndex()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChange;
    }

    void Map::IncreaseChangeIndex()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChange++;
    }

    int Map::GetLastMapChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChangeNotified;
    }

    void Map::SetLastMapChange(int currentChangeId)
    {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChangeNotified = currentChangeId;
    }

    void Map::PreSave(std::set<GeometricCamera *> &spCams)
    {
        int nMPWithoutObs = 0;

        std::set<MapPoint *> tmp_mspMapPoints1;
        tmp_mspMapPoints1.insert(mspMapPoints.begin(), mspMapPoints.end());

        for (MapPoint *pMPi : tmp_mspMapPoints1)
        {
            if (!pMPi || pMPi->isBad())
                continue;

            if (pMPi->GetObservations().size() == 0)
            {
                nMPWithoutObs++;
            }
            map<KeyFrame *, std::tuple<int, int>> mpObs = pMPi->GetObservations();
            for (map<KeyFrame *, std::tuple<int, int>>::iterator it = mpObs.begin(), end = mpObs.end(); it != end; ++it)
            {
                if (it->first->GetMap() != this || it->first->isBad())
                {
                    pMPi->EraseObservation(it->first);
                }
            }
        }

        // Saves the id of KF origins
        mvBackupKeyFrameOriginsId.clear();
        mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
        for (int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
        {
            mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
        }

        // Backup of MapPoints
        mvpBackupMapPoints.clear();

        std::set<MapPoint *> tmp_mspMapPoints2;
        tmp_mspMapPoints2.insert(mspMapPoints.begin(), mspMapPoints.end());

        for (MapPoint *pMPi : tmp_mspMapPoints2)
        {
            if (!pMPi || pMPi->isBad())
                continue;

            mvpBackupMapPoints.push_back(pMPi);
            pMPi->PreSave(mspKeyFrames, mspMapPoints);
        }

        // Backup of KeyFrames
        mvpBackupKeyFrames.clear();
        for (KeyFrame *pKFi : mspKeyFrames)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            mvpBackupKeyFrames.push_back(pKFi);
            pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
        }

        mnBackupKFinitialID = -1;
        if (mpKFinitial)
        {
            mnBackupKFinitialID = mpKFinitial->mnId;
        }

        mnBackupKFlowerID = -1;
        if (mpKFlowerID)
        {
            mnBackupKFlowerID = mpKFlowerID->mnId;
        }
    }

    void Map::PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera *> &mpCams)
    {
        std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
        std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

        map<long unsigned int, MapPoint *> mpMapPointId;
        for (MapPoint *pMPi : mspMapPoints)
        {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->UpdateMap(this);
            mpMapPointId[pMPi->mnId] = pMPi;
        }

        map<long unsigned int, KeyFrame *> mpKeyFrameId;
        for (KeyFrame *pKFi : mspKeyFrames)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateMap(this);
            pKFi->SetORBVocabulary(pORBVoc);
            pKFi->SetKeyFrameDatabase(pKFDB);
            mpKeyFrameId[pKFi->mnId] = pKFi;
        }

        // References reconstruction between different instances
        for (MapPoint *pMPi : mspMapPoints)
        {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
        }

        for (KeyFrame *pKFi : mspKeyFrames)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
            pKFDB->add(pKFi);
        }

        if (mnBackupKFinitialID != -1)
        {
            mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
        }

        if (mnBackupKFlowerID != -1)
        {
            mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
        }

        mvpKeyFrameOrigins.clear();
        mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
        for (int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
        {
            mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
        }

        mvpBackupMapPoints.clear();
    }

}
