#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas, double segProbThreshold, int minCloudSize,
                                               std::pair<float, float> distFilterThreshold, float downsampleLeafSize)
    {
        mpAtlas = pAtlas;
        mMinCloudSize = minCloudSize;
        mSegProbThreshold = segProbThreshold;
        mDistFilterThreshold = distFilterThreshold;
        mDownsampleLeafSize = downsampleLeafSize;
        mPlanePoseMat = Eigen::Matrix4f::Identity();
    }

    void SemanticSegmentation::Run()
    {
        while (true)
        {
            // Check if there are new KeyFrames in the buffer
            if (segmentedImageBuffer.empty())
            {
                usleep(3000);
                continue;
            }

            // retrieve the oldest one
            mMutexNewKFs.lock();
            std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> segImgTuple = segmentedImageBuffer.front();
            segmentedImageBuffer.pop_front();
            mMutexNewKFs.unlock();

            // separate point clouds while applying threshold
            pcl::PCLPointCloud2::Ptr pclPc2SegPrb = std::get<2>(segImgTuple);
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clsCloudPtrs;
            threshSeparatePointCloud(pclPc2SegPrb, clsCloudPtrs);

            // get the point cloud from the respective keyframe via the atlas - ignore it if KF doesn't exist
            KeyFrame *thisKF = mpAtlas->GetKeyFrameById(std::get<0>(segImgTuple));
            if (thisKF == nullptr)
                continue;
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud = thisKF->getCurrentFramePointCloud();

            // fill in class specific point clouds with XYZZ and RGB from the keyframe pointcloud
            enrichClassSpecificPointClouds(clsCloudPtrs, thisKFPointCloud);

            // get all planes for each class specific point cloud using RANSAC
            std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clsPlanes =
                getPlanesFromClassClouds(clsCloudPtrs, mMinCloudSize);

            // set the class specific point clouds to the keyframe
            thisKF->setCurrentClsCloudPtrs(clsCloudPtrs);

            // Add the planes to Atlas
            addPlanesToAtlas(thisKF, clsPlanes);
        }
    }

    void SemanticSegmentation::AddSegmentedFrameToBuffer(std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        segmentedImageBuffer.push_back(*tuple);
    }

    std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>> SemanticSegmentation::GetSegmentedFrameBuffer()
    {
        return segmentedImageBuffer;
    }

    void SemanticSegmentation::threshSeparatePointCloud(
        pcl::PCLPointCloud2::Ptr &pclPc2SegPrb, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs)
    {
        const int width = pclPc2SegPrb->width;
        const int numPoints = width * pclPc2SegPrb->height;
        const int pointStep = pclPc2SegPrb->point_step;
        const int numClasses = pointStep / bytesPerClassProb;

        for (int i = 0; i < numClasses; i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pointCloud->is_dense = false;
            pointCloud->height = 1;
            clsCloudPtrs.push_back(pointCloud);
        }

        // parse the point cloud message and apply thresholding
        const uint8_t *data = pclPc2SegPrb->data.data();
        for (int i = 0; i < numPoints; i++)
        {
            for (int j = 0; j < numClasses; j++)
            {
                float value;
                memcpy(&value, data + pointStep * i + bytesPerClassProb * j + pclPc2SegPrb->fields[0].offset, bytesPerClassProb);

                if (value >= mSegProbThreshold)
                {
                    // inject coordinates as a point to respective point cloud
                    pcl::PointXYZRGB point;
                    point.y = static_cast<int>(i / width);
                    point.x = i % width;
                    clsCloudPtrs[j]->push_back(point);
                }
            }
        }

        // specify size/width and log statistics
        for (int i = 0; i < numClasses; i++)
        {
            clsCloudPtrs[i]->width = clsCloudPtrs[i]->size();
            clsCloudPtrs[i]->header = pclPc2SegPrb->header;
        }
    }

    void SemanticSegmentation::enrichClassSpecificPointClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &thisKFPointCloud)
    {
        for (int i = 0; i < clsCloudPtrs.size(); i++)
        {
            for (int j = 0; j < clsCloudPtrs[i]->width; j++)
            {
                const pcl::PointXYZRGB point = thisKFPointCloud->at(clsCloudPtrs[i]->points[j].x, clsCloudPtrs[i]->points[j].y);
                clsCloudPtrs[i]->points[j] = pcl::PointXYZRGB(point);
            }
            clsCloudPtrs[i]->header.frame_id = thisKFPointCloud->header.frame_id;
        }
    }

    std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clsCloudPtrs, int minCloudSize)
    {
        std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> clsPlanes;

        // downsample/filter the pointcloud and extract planes
        for (int i = 0; i < clsCloudPtrs.size(); i++)
        {
            // [TODO] decide when to downsample and/or distance filter

            // Downsample the given pointcloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud = Utils::pointcloudDownsample<pcl::PointXYZRGB>(clsCloudPtrs[i], mDownsampleLeafSize);

            // Filter the pointcloud based on a range of distance
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud = Utils::pointcloudDistanceFilter(downsampledCloud, mDistFilterThreshold);

            // copy the filtered cloud for later storing into the keyframe
            pcl::copyPointCloud(*filteredCloud, *clsCloudPtrs[i]);

            std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> extractedPlanes;
            if (filteredCloud->points.size() > minCloudSize)
            {
                extractedPlanes = Utils::ransacPlaneFitting(filteredCloud, minCloudSize);
            }
            clsPlanes[i] = extractedPlanes;
        }
        return clsPlanes;
    }

    void SemanticSegmentation::addPlanesToAtlas(KeyFrame *pKF,
                                                std::unordered_map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> &clsPlanes)
    {
        for (int clsId = 0; clsId < clsPlanes.size(); clsId++)
        {
            for (auto planePoint : clsPlanes[clsId])
            {
                // Get the plane equation from the points
                Eigen::Vector4d planeEstimate(planePoint->back().normal_x, planePoint->back().normal_y,
                                              planePoint->back().normal_z, planePoint->back().curvature);
                g2o::Plane3D detectedPlane(planeEstimate);
                // Convert the given plane to global coordinates
                g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                             detectedPlane);

                // Check if we need to add the wall to the map or not
                int matchedPlaneId = Utils::associatePlanes(mpAtlas->GetAllPlanes(), globalEquation);

                // [TODO] - Add flags to decide whether SemSeg can add planes or it just updates
                if (matchedPlaneId == -1)
                {
                    // [TODO] - Add a flag to decide whether SemSeg runs indepedently or with GeoSeg
                    // cout << "No matched plane found." << endl;
                    continue;
                }
                else
                {
                    std::string planeType = clsId ? "Wall" : "Floor";
                    // cout << "Matched a plane! Updating the plane type to " << planeType << endl;
                    updateMapPlane(matchedPlaneId, clsId);
                }
            }
        }
    }

    void SemanticSegmentation::createMapPlane(ORB_SLAM3::KeyFrame *pKF, const g2o::Plane3D estimatedPlane, int clsId,
                                              const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud)
    {
        ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
        newMapPlane->setColor();
        newMapPlane->setLocalEquation(estimatedPlane);
        newMapPlane->SetMap(mpAtlas->GetCurrentMap());
        newMapPlane->addObservation(pKF, estimatedPlane);
        newMapPlane->setId(mpAtlas->GetAllPlanes().size());

        // Set the plane type to the semantic class
        newMapPlane->setPlaneType(Utils::getPlaneTypeFromClassId(clsId));

        // Set the global equation of the plane
        g2o::Plane3D globalEquation = Utils::convertToGlobalEquation(pKF->GetPoseInverse().matrix().cast<double>(),
                                                                     estimatedPlane);
        newMapPlane->setGlobalEquation(globalEquation);

        // Fill the plane with the pointcloud
        if (!planeCloud->points.empty())
            newMapPlane->setMapClouds(planeCloud);
        else
            // Loop to find the points lyinupdateMapPlane(pKF, detectedPlane, planePog on wall
            for (const auto &mapPoint : mpAtlas->GetAllMapPoints())
                if (Utils::pointOnPlane(newMapPlane->getGlobalEquation().coeffs(), mapPoint))
                    newMapPlane->setMapPoints(mapPoint);

        pKF->AddMapPlane(newMapPlane);
        mpAtlas->AddMapPlane(newMapPlane);
    }

    void SemanticSegmentation::updateMapPlane(int planeId, int clsId)
    {
        // retrieve the plane from the map
        Plane *matchedPlane = mpAtlas->GetPlaneById(planeId);

        // plane type compatible with the Plane class
        ORB_SLAM3::Plane::planeVariant planeType = Utils::getPlaneTypeFromClassId(clsId);

        // get the floor plane from the atlas
        Plane *floorPlane = mpAtlas->GetFloorPlane();
        
        // filter the floor pointcloud maintaining only one floor plane
        // [TODO] - Add check whether the floor plane was updated or not? to skip repeating this process
        if (floorPlane != nullptr)
        {
            // transform the planeCloud according to the planePose
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr planeCloud = floorPlane->getMapClouds();
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::transformPointCloud(*planeCloud, *transformedCloud, mPlanePoseMat);

            // print the standard deviation across the x and y axes of the transformedCloud
            std::vector<float> yVals;
            for (const auto &point : transformedCloud->points)
            {
                yVals.push_back(point.y);
            }

            // get an estimate of the numPoint-th lower point - sort descending
            uint8_t percentile = 10;
            int numPoint = (percentile * yVals.size()) / 100;
            std::partial_sort(yVals.begin(), yVals.begin() + numPoint, yVals.end(), std::greater<float>());
            float maxY = yVals[numPoint - 1];

            // define the threshold for the floor plane
            // [TODO] - Parameterize threshold
            const float threshY = maxY - 0.30;

            // filter the cloud based on threshold
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

            std::copy_if(transformedCloud->begin(),
                transformedCloud->end(),
                std::back_inserter(filteredCloud->points),
                [&](const pcl::PointXYZRGBNormal &p)
                {
                return p.y > threshY;
                });
            filteredCloud->height = 1;
            filteredCloud->is_dense = false;                    

            if (filteredCloud->points.size() > 0)
            {
                // replace the planeCloud with the filteredCloud after performing inverse transformation
                pcl::transformPointCloud(*filteredCloud, *transformedCloud, mPlanePoseMat.inverse());
                floorPlane->replaceMapClouds(transformedCloud);
            }
            // cout << "Points in floor plane: " << filteredCloud->points.size() << endl;
        }

        // recompute the transformation from floor to horizontal
        if (planeType == ORB_SLAM3::Plane::planeVariant::FLOOR)
        {
            // when the floor is found for the first time
            if (floorPlane == nullptr)
            {
                matchedPlane->setPlaneType(planeType);
                mpAtlas->setFloorPlaneId(planeId);
            }
            
            // get the floor plane from atlas
            Plane *plane = mpAtlas->GetFloorPlane();

            // update the mPlanePoseMat
            // initialize the transformation with translation set to a zero vector
            Eigen::Isometry3d planePose;
            planePose.translation() = Eigen::Vector3d(0, 0, 0);

            // normalize the normal vector
            Eigen::Vector3d normal = plane->getGlobalEquation().coeffs().head<3>();
            normal.normalize();

            // get the rotation from the floor plane to the plane with y-facing vertical downwards
            Eigen::Vector3d verticalAxis = Eigen::Vector3d(0, -1, 0);
            Eigen::Quaterniond q;
            q.setFromTwoVectors(normal, verticalAxis);
            planePose.linear() = q.toRotationMatrix();

            // form homogenous transformation matrix
            Eigen::Matrix4f planePoseMat = planePose.matrix().cast<float>();
            planePoseMat(3, 3) = 1.0;
            mPlanePoseMat = planePoseMat;
        }

        // add semantic information to a wall plane if the plane orientation is good
        if (planeType == ORB_SLAM3::Plane::planeVariant::WALL)
        {
            // transform the matchedCloud according to the planePose
            // only works if the floor plane is set, needs the correction matrix: mPlanePoseMat
            if (floorPlane != nullptr)
            {
                // calculate the transformed equation
                Eigen::Matrix3f rotationMatrix = mPlanePoseMat.block<3, 3>(0, 0);

                // Compute the inverse transpose of the rotation matrix
                Eigen::Matrix3f inverseTransposeRotationMatrix = rotationMatrix.inverse().transpose();

                // Extract the coefficients of the original plane equation
                Eigen::Vector4d originalPlaneCoefficients = matchedPlane->getGlobalEquation().coeffs();

                // Transform the coefficients of the plane equation
                Eigen::Vector3f transformedPlaneCoefficients;
                transformedPlaneCoefficients.head<3>() = inverseTransposeRotationMatrix * originalPlaneCoefficients.head<3>().cast<float>();

                // normalize the transformed coefficients
                transformedPlaneCoefficients.normalize();

                // if the transformed plane is vertical based on absolute value, then assign semantic, otherwise ignore
                // [TODO] - Parameterize threshold
                if (abs(transformedPlaneCoefficients(1)) < 0.18)
                    matchedPlane->setPlaneType(planeType);
            }
        }
    }
}