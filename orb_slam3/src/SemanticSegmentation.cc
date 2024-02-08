#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
    SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas, double segProbThreshold)
    {
        mpAtlas = pAtlas;
        mSegProbThreshold = segProbThreshold;
    }

    void SemanticSegmentation::Run()
    {
        while (1)
        {
            // Check if there are new KeyFrames in the buffer
            if (segmentedImageBuffer.empty())
            {
                usleep(3000);
                continue;
            }

            // retrieve the oldest one
            mMutexNewKFs.lock();
            std::pair<cv::Mat, pcl::PCLPointCloud2::Ptr> segImgPair = segmentedImageBuffer.front();
            segmentedImageBuffer.pop_front();
            mMutexNewKFs.unlock();

            // separate point clouds while applying threshold
            pcl::PCLPointCloud2::Ptr pclPc2SegPrb = segImgPair.second;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clsCloudPtrs;
            threshSeparatePointCloud(pclPc2SegPrb, clsCloudPtrs);

            // structure below
            // Get the unfiltered point cloud of the KF
            // Get the probability map for the pointcloud from Semantic Segmenter
            // For probabilities > 0.9, as the wall, segment all the points in the point cloud
            // Return the pointcloud with only walls and floors
            // Perform distance filter and downsample
            // Send it to the Ransac and get the plane equations
            // Do the association with the planes in Geometric
            // usleep(3000);
        }
    }

    void SemanticSegmentation::AddSegmentedFrameToBuffer(std::pair<cv::Mat, pcl::PCLPointCloud2::Ptr> *pair)
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        segmentedImageBuffer.push_back(*pair);
    }

    std::list<std::pair<cv::Mat, pcl::PCLPointCloud2::Ptr>> SemanticSegmentation::GetSegmentedFrameBuffer()
    {
        return segmentedImageBuffer;
    }

    void SemanticSegmentation::threshSeparatePointCloud(
        pcl::PCLPointCloud2::Ptr &pclPc2SegPrb, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clsCloudPtrs)
    {
        // // only for validation - needs Mat image as input
        // cv::Mat image = cv_imgSeg->image;
        // cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

        const int width = pclPc2SegPrb->width;
        const int numPoints = width * pclPc2SegPrb->height;
        const int pointStep = pclPc2SegPrb->point_step;
        const int numClasses = pointStep/bytesPerClassProb;

        cout << numClasses << " " << numPoints << " " << pointStep << endl;
        
        for (int i = 0; i < numClasses; i++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pointCloud->is_dense = false;
            pointCloud->height = 1;
            clsCloudPtrs.push_back(pointCloud);
        }

        // read the point cloud and apply thresholding
        const uint8_t* data = pclPc2SegPrb->data.data();
        for (int i = 0; i < numPoints; i++) {
            for (int j = 0; j < numClasses; j++) {
                float value;
                memcpy(&value, data + pointStep * i + bytesPerClassProb * j + pclPc2SegPrb->fields[0].offset, bytesPerClassProb);
                
                if (value >= 0.9){
                    // inject coordinates as a point to respective point cloud
                    pcl::PointXYZ point;
                    point.y = static_cast<int>(i / width);
                    point.x = i % width;
                    clsCloudPtrs[j]->push_back(point);

                    // // only for validation
                    // if (j == 1 && point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows) {
                    // // Set the corresponding pixel in the mask to white (255)
                    //     mask.at<uchar>(point.y, point.x) = 255;
                    // }
                }
                
                // Occasionaly log values
                if (i%100000 == 0) cout << "Point " << i << ", Class " << j << ": " << value << endl;
            }
        }

        // specify size/width and log statistics
        for (int i = 0; i < numClasses; i++){
            clsCloudPtrs[i]->width = clsCloudPtrs[i]->size();
            cout << "Class " << i << " has " << clsCloudPtrs[i]->width << " points." << endl;
        }

        // // only for validation
        // cv::imshow("Image", image);
        // cv::imshow("Mask", mask);
        // cv::waitKey(10);
    }
}