#include "QualityAssessment.h"

// Constructor
QualityAssessment::QualityAssessment()
{
}


// Check Quality of one finger
SFPQA QualityAssessment::sharpnessSobelGrad(SFinger finger)
{
    SFPQA fpqa;
    fpqa.mFinger = finger;
    fpqa.mSharpnessContrastSobelGrad = 0.0;

    // norm size
    cv::Mat imagePatch;
#if 1
    int normWidth = 300;
    float scaleFactor = (float)normWidth/(float)finger.fingerPatch.cols;
    if (scaleFactor < 0.5)
    {
        scaleFactor = 0.5;
    }
//    std::cout << "scaleFactor: " << scaleFactor << std::endl;
    cv::resize(finger.fingerPatch, imagePatch, cv::Size(), scaleFactor, scaleFactor);
#else
    imagePatch = finger.fingerPatch.clone();
#endif

    cv::Mat blurredImage;
    cv::GaussianBlur( imagePatch, blurredImage, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

    cv::Mat gray;
    cv::cvtColor( blurredImage, gray, CV_RGB2GRAY );

    cv::Mat grad;
    sobelEdgeStrength(imagePatch, grad);

    // bin image
    cv::Mat imageBin;
    imageBin = grad > 50;

    // count edges & normalize
    int numPx = imagePatch.size().width * imagePatch.size().height;
    unsigned char* binImage = imageBin.data;
    int edgeCount=0;
    for(int i=0; i<numPx; i++)
    {
        if(binImage[i]==255)
        {
            edgeCount++;
        }
    }
    fpqa.mSharpnessContrastSobelGrad = (float)edgeCount/(float)numPx; // normalize

//    if(fpqa.mSharpnessContrastSobelGrad>mOneFinger.mSharpnessContrastSobelGrad)
//    {
//        mOneFinger.mSharpnessContrastSobelGrad = fpqa.mSharpnessContrastSobelGrad;
//        mOneFinger.mFinger = finger.at(0);
//    }

    return fpqa;
}



std::vector<int> QualityAssessment::sortDesc(std::vector<float> unsortedList)
{
    std::vector<int> idxListSorted;
    for(int k=0; k<unsortedList.size(); k++)
    {
        bool Sorted=false;
        for(int i=0; i<idxListSorted.size(); i++)
        {
            if(unsortedList.at(k) > unsortedList.at(idxListSorted.at(i)))
            {
                idxListSorted.insert(idxListSorted.begin()+i, k);
                Sorted = true;
                break;
            }
        }
        if(!Sorted)
        {
            idxListSorted.push_back(k);
        }
    }

    return idxListSorted;
}
