#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include "CFingerprintExtractor.h"
#include "SFinger.h"

#include <opencv2/opencv.hpp>


// -------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Setup camera
    ///
    int videoCaptureHandle = 0;
    VideoCapture cap(videoCaptureHandle); // 0: opens the default camera, 1: other
    if(!cap.isOpened())  // check if we succeeded
    {
        cout << "Could not open camera ...";
        return -1;
    }
    cv::Mat inFrame;
    cap >> inFrame; // Capture one frame
    CFingerprintExtractor fingerprintExtraction(inFrame);

    while (1)
    {
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        ///
        /// Capture one frame
        ///
        ///
        cap >> inFrame;
        ///
        ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////



        /////////////////////////////////////////////////////////////////////////////////////////////////////
        ///
        /// Fingerprint detection
        ///
        ///
        std::vector<SFinger> fingersPalm = fingerprintExtraction.doAllTheWork(inFrame);
        cv::Mat inFrameResult = inFrame.clone();
        fingerprintExtraction.renderResults(inFrameResult, fingersPalm);
        cv::imshow("results", inFrameResult);
        ///
        ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////


        cv::waitKey(1);
    }
    return 0;
}



