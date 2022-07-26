#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "FingerprintVerification.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <fstream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>
#include <opencv2/core/utility.hpp>
#include <chrono>
#include <iomanip>
using namespace std;
using namespace cv;


// -------------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Setup camera
    ///
    //int videoCaptureHandle = 0;
    VideoCapture cap(0); // 0: opens the default camera, 1: other
    if(!cap.isOpened ())  // check if we succeeded
    {
        cout << "Could not open camera ...";
        return -1;
    }

    //std::cout << "Camera Init Successful\n";
    //std::cout << "Setting parameters..\n";


    std::cout <<"\t[PARAM_FPS] ";
    if(!cap.set (CAP_PROP_FPS,15)){
         std::cout <<" SUCCESS\n";
    }else{
         std::cout <<"FAIL\n";
    }
    std::cout <<"\t[PARAM_FRAME_WIDTH] ";
    if(!cap.set (CAP_PROP_FRAME_WIDTH, 280)){
         std::cout <<"SUCCESS\n";
    }else{
         std::cout <<"FAIL\n";
    }
    std::cout <<"\t[PARAM_FRAME_HEIGHT] ";
    if(!cap.set (CAP_PROP_FRAME_HEIGHT,480)){
        std::cout <<"SUCCESS\n";
    }else{
        std::cout <<"FAIL\n";
    }

    double nFPS = cap.get (CAP_PROP_FPS);
    std::cout <<"Loaded FPS : "<< nFPS << "\n";
//    namedWindow("show", 1);
//    for(;;)
//    {
       cv::Mat Frame;
       cap >> Frame; // Capture one frame
       // imshow ("show", Frame);
       //if (waitKey (1) >= 0) break;

    ///
    /////////////////////////////////////////////////////////////////////////////////////////////////////


    FingerprintVerification fpVer(Frame);

    int maxBatchSize = 50;  // max number of images per batch
    int minBatchSize = 10;  // min number of images per batch
    unsigned int batchCount = 0;

    while (1)
    {
        std::cout << "Batch nbr.: " << batchCount++ << std::endl;


        /////////////////////////////////////////////////////////////////////////////////////////////////////
        ///
        ///  Get single snapshots
        ///
        ///  Call the init function first (delete lists, clear all the data)
        fpVer.newBatchInit();
        for (int i=0; i<maxBatchSize; i++)
        {
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ///
            ///  Capture one frame
            ///
            cap >> Frame;
            ///
            ///
            /////////////////////////////////////////////////////////////////////////////////////////////////////


            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ///
            ///  Fingerprint detection for one frame
            ///
            bool fingerprintExtractorDbg = true;
            fpVer.getFingerprints(Frame, fingerprintExtractorDbg);
            ///
            ///
            /////////////////////////////////////////////////////////////////////////////////////////////////////

            cv::waitKey(30);
        }
        ///
        ///  Possible fingerprint images are now available in mFingerList
        ///
        ////////////////////////////////////////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///
        /// If fingerprints are captured within last batch, carry out the next steps ...
        ///
        ///
        if (fpVer.getNumFingerprints() >= minBatchSize)
        {
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ///
            ///  Fingerprint quality assessment
            ///
            ///
            bool qualityAssessmentDbg = true;
            SFPQA bestFingerprint = fpVer.qualityAssessment(qualityAssessmentDbg);
#if 1
            std::cout << "Max sharpness " << bestFingerprint.mSharpnessContrastSobelGrad << std::endl;
            cv::imshow("Best Fingerprint", bestFingerprint.mFinger.fingerPatch);
#endif
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ///
            ///  Fingerprint verification


            //cv::Mat img_2 = imread("D:\thesis\uss_finger\FingerprintVerification\Right_index_fig_1.jpeg", cv::IMREAD_COLOR);
            cv::Mat img_2 = imread("/home/pi/thesis/FingerprintVerification/Right_index_fig_1.jpeg", cv::IMREAD_COLOR);
             if(  !img_2.data )
                  {
                      std::cout<< " --(!) Error reading images " << std::endl; return -1;
                  }

            cv::cvtColor( bestFingerprint.mFinger.fingerPatch, bestFingerprint.mFinger.fingerPatch, cv::COLOR_BGR2GRAY );
            cv::Mat dst1;
            equalizeHist( bestFingerprint.mFinger.fingerPatch, dst1 );
            //imshow( "Source image", bestFingerprint.mFinger.fingerPatch );
            imshow( "Equalized Image", dst1 );
            Size size1 = { 154,203 };
              resize(bestFingerprint.mFinger.fingerPatch, bestFingerprint.mFinger.fingerPatch,size1);
            cv::cvtColor( img_2, img_2, cv::COLOR_BGR2GRAY );
            cv::Mat dst2;
            equalizeHist( img_2, dst2 );
            // imshow( "Source image1", img_2 );
             imshow( "Equalized Image1", dst2 );
             Size size = { 154,203 };
               resize(img_2, img_2,size);
               //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
                  int minHessian = 400;
                  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
                  detector->setHessianThreshold(minHessian);
                  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
                  cv::Mat descriptors_1, descriptors_2;
                  detector->detectAndCompute( dst1, cv::Mat(), keypoints_1, descriptors_1 );
                  detector->detectAndCompute( dst2, cv::Mat(), keypoints_2, descriptors_2 );

                  //-- Step 2: Matching descriptor vectors using FLANN matcher
                  cv::FlannBasedMatcher matcher;
                  std::vector< cv::DMatch > matches;
                  matcher.match( descriptors_1, descriptors_2, matches );

                  double max_dist = 0;
                  double min_dist = 100;

                  //-- Quick calculation of max and min distances between keypoints
                  for( int i = 0; i < descriptors_1.rows; i++ )
                  {
                    double dist = matches[i].distance;
                      if( dist < min_dist )
                          min_dist = dist;
                      if( dist > max_dist )
                          max_dist = dist;
                  }
                  printf("-- Max dist : %f \n", max_dist );
                  printf("-- Min dist : %f \n", min_dist );
                  //-- Draw only "good" matches (i.e. whose distance is less than 'thresholdGoodMatch',
                  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very small)
                  //-- PS.- radiusMatch can also be used here.
                  std::vector<cv::DMatch> good_matches;

                  double thresholdGoodMatch = 0.42; // <-- IMPORTANT VARIABLE!! 0.1 ... 0.2
                  for( int i = 0; i < descriptors_1.rows; i++ )
                  {
                    if( matches[i].distance <= cv::max(thresholdGoodMatch, 0.02))
                    {
                        good_matches.push_back( matches[i]);
                    }
                  }
                  //-- Draw only "good" matches
                  cv::Mat img_matches;
                  cv::drawMatches(  dst1, keypoints_1, dst2, keypoints_2,
                                  good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                                  std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
                               );
                  //-- Show detected matches
                  cv::imshow( "Good Matches", img_matches );
                  for( int i = 0; i < (int)good_matches.size();i++ )
                  {
                    printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );

                  }


                  //--Matching score
                 double m = ((float)(int)(good_matches.size()-1)) /((float)std::max(good_matches[(int)(good_matches.size()-1)].queryIdx,good_matches[(int)(good_matches.size()-1)].trainIdx));
                   printf(  "-- Matching score:  [%f]\n",m);

                   if ( m > 0.8 ) {
                        printf("-- Fingerprint matched\n");
                     }
                     else if ( m < 0.8 ) {
                        printf("-- Fingerprint did not match\n");
                     }


            ///
            ///
            /////////////////////////////////////////////////////////////////////////////////////////////////////
                   cv::waitKey(0);
                   return 0;
         }
}
}






