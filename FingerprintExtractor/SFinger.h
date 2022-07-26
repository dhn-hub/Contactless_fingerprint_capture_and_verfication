#ifndef SFINGER_H
#define SFINGER_H

#include <opencv2/opencv.hpp>

struct SFinger
{
    SFinger()
    {
        roiRot = cv::RotatedRect(cv::Point2f(0,0),cv::Size2f(0,0),0.0);
        roi = cv::Rect(0,0,0,0);
        roiInMainImage = cv::Rect(0,0,0,0);
        fingerRoot = cv::Point(0,0);
        fingerTip = cv::Point(0,0);
    }

//    SFinger(SFinger other)
//    {
//        roiRot = other.roiRot;
//        roi = other.roi;
//        roiInMainImage = other.roiInMainImage;
//        fingerRoot = other.fingerRoot;
//        fingerTip = other.fingerTip;
//        fingerPatch = other.fingerPatch.clone();
//    }

    SFinger& operator=(const SFinger& other)
    {
        roiRot = other.roiRot;
        roi = other.roi;
        roiInMainImage = other.roiInMainImage;
        fingerRoot = other.fingerRoot;
        fingerTip = other.fingerTip;
        fingerPatch = other.fingerPatch.clone();

        return *this;
    }

    //  EdgeMatcherEdgePair edgePair;
    cv::RotatedRect roiRot;
    cv::Rect roi;
    cv::Rect roiInMainImage;
    cv::Point fingerRoot;
    cv::Point fingerTip;
    cv::Mat fingerPatch;
};


#endif // SFINGER_H
