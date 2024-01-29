#include "GeometricSegmentation.h"

namespace ORB_SLAM3
{
    GeometricSegmentation::GeometricSegmentation(Atlas *pAtlas)
    {
        mpAtlas = pAtlas;
    }

    void GeometricSegmentation::AddKeyFrameToBuffer(KeyFrame *pKF)
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        mvpKeyFrameBuffer.push_back(pKF);
    }

    std::list<KeyFrame *> GeometricSegmentation::GetKeyFrameBuffer()
    {
        return mvpKeyFrameBuffer;
    }

    void GeometricSegmentation::Run()
    {
        while (1)
        {
            // Check if there are new KeyFrames in the buffer
            if (mvpKeyFrameBuffer.empty())
                continue;

            // Get the first KeyFrame in the buffer
            mMutexNewKFs.lock();
            KeyFrame *mpCurrentKeyFrame = mvpKeyFrameBuffer.front();
            mvpKeyFrameBuffer.pop_front();
            mMutexNewKFs.unlock();

            // For each currentKeyFrame do plane segmentation (RANSCA) and get plane parameters  {
            // Another loop to pop it one by one
            // Check if there are new keyframes in the buffer
            // Apply RANSAC
            // Add the new planes to the KF
            usleep(3000);
        }
    }
}