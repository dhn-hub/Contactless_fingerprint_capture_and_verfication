#include "CFingerprintExtractor.h"

///
/// \brief CFingerprintExtractor::CFingerprintExtractor
/// \param inFrame
///
CFingerprintExtractor::CFingerprintExtractor(cv::Mat inFrame)
{

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///  PARAMETER
    ///  image scaling
    ///
    int numRowsScaledImg = 640.0; // PARA
    mScaleFactor = numRowsScaledImg/inFrame.cols;
    cv::resize(inFrame, mInFrameScaled, cv::Size(), mScaleFactor, mScaleFactor);
    mpEdgeMatcher = new CEdgeMatcher(mInFrameScaled);



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// PARAMETERS: Edge Matcher
    ///
    mpEdgeMatcher->getParameters()->CONTOUR_MIN_TOTAL_POINTS = 50;        //!< Minimum number of points within a contour
    mpEdgeMatcher->getParameters()->CONTOUR_SMOOTH_POINTS = 10;          //!< Number of pixels used for averaging angles, ...
    mpEdgeMatcher->getParameters()->CONTOUR_BORDER_CROP = 10;            //!< Minimaler Abstand von Geradensegmenten zum Bildrand, um zur Rekonstruktion von Kanten verwendet zu werden

    mpEdgeMatcher->getParameters()->LINE_MAX_ANGLE_DEVIATION = 20.0;//10.0;   //!< Maximum angle deviation within a line segment
    mpEdgeMatcher->getParameters()->LINE_MAX_ANGLE_DEVIATION_COUNT = 5;//25;     //!< Maximum number of consecutive pixels with too high angle deviation within a line segment
    mpEdgeMatcher->getParameters()->LINE_MIN_LENGTH = 10;                   //!< Minimal length of a line segment
    mpEdgeMatcher->getParameters()->LINE_MAX_CURVINESS = 5;//30;               //!< Maximum curviness of a line segment
    mpEdgeMatcher->getParameters()->LINE_MAX_REFLECTION_RATIO = 0.5;      //!< The maximum ratio of a segment's contour points classified as reflection

    mpEdgeMatcher->getParameters()->EDGE_MAX_SEGMENT_ANGLE = 12.5;                //!< Maximum angle between an edge and a line segment possibly used for refining the edge
    mpEdgeMatcher->getParameters()->EDGE_MAX_SEGMENT_DISTANCE_LENGTH_RATIO = 2.0;     //!< Maximum ratio of a segment's length and its endpoint distance
    mpEdgeMatcher->getParameters()->EDGE_MAX_SEGMENT_RELATIVE_AXIS_DISTANCE = 10;    //!< Maximum relative distance of line segment endpoints to edge axis for edge refinement
    mpEdgeMatcher->getParameters()->EDGE_MIN_LENGTH = 10;                           //!< Minimale L&auml;nge einer Kante

    mpEdgeMatcher->getParameters()->EDGEPAIR_MAX_EDGE_PAIRING_ANGLE = 30.0;        //!< Maximaler zwischen Kanten zulaessiger Winkel zur Paarung zu einem Tool
    mpEdgeMatcher->getParameters()->EDGEPAIR_MIN_EDGE_DISTANCE = 2.0;              //!< Minimaler Abstand zweier paralleler Kanten, die gepaart werden koennen
    mpEdgeMatcher->getParameters()->EDGEPAIR_MIN_BLOB_FILLING = 0.75;              //!< Minimum ratio white pixels required on a blob surface
    mpEdgeMatcher->getParameters()->EDGEPAIR_MIN_LENGTH = 40;//80;                 //!< Minimum length of an edge pair (distance between contourTips)
    // ----------------------------------------------------------------------------------------------------------------

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///  OTHER PARAMETERS
    ///
    mNumMaxFinger = 1;                  // max. nbr. of finger
    mMaxAngleFinger = 80.0;             // allowed orientation of finger (here: only upright fingers)
    mOffsetLenForWidth = 150.0;         // offest value from fingertip for determining the fingerwidth
    mMaxIterFingerWidth = 400;          // max. iter. for determining the fingerwidth
    mRatioFingerHeight2Width = 1.5;     // ratio of fingerwidth for determinig the length of an fingerprint
    mMinFingerWidth = 50;//100;         // for checking the min. fingerwidth
    mMaxFingerWidth = 700;              // for checking the max. fingerwidth

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief CFingerprintExtractor::doAllTheWork
/// \param inFrame                  Input frame
/// \return std::vector<SFinger>    List of detected fingerprints
///
std::vector<SFinger> CFingerprintExtractor::doAllTheWork(cv::Mat inFrame)
{

    vector<SFinger> fingersPalm;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// resize inFrame -> fingerprint detection is performed on reduced size
    ///                   but fingerprint cropping is done from original size image
    ///
    cv::resize(inFrame, mInFrameScaled, cv::Size(), mScaleFactor, mScaleFactor);
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Palm Segmentation
    /// convert to 3-plane image first
    /// 'mask_image': bin image
    ///
//    CPalmSegmentation palmSegmentation;
    cv::Mat skinColorMask_ocv(mInFrameScaled.rows, mInFrameScaled.cols, CV_8UC1);
    int BlobSize=0;
    mPalmSegmentation.getSkinColorMask(mInFrameScaled, skinColorMask_ocv, BlobSize);
    if (0) // debugging
    {
        std::cout << BlobSize << std::endl;
        cv::imshow("mask", skinColorMask_ocv);
    }
    ///
    ///    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    if (BlobSize > 0)
    {
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///
        /// Edge Matcher
        ///
        mpEdgeMatcher->extractEdgePairsFromImage(skinColorMask_ocv);
        if (0)  // for debugging
        {
            mpEdgeMatcher->renderResults();
            imshow("Contour Image", mpEdgeMatcher->contourImage);
            imshow("Edge Image", mpEdgeMatcher->edgeImage);
        }
        ///
        ///
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///
        /// for each finger:
        /// get fingerprint ROI, fingerprint crop, ...
        ///
        int nFingers = mpEdgeMatcher->edgePair.size();
        if (nFingers > mNumMaxFinger)
            nFingers = mNumMaxFinger;

        for (int iFinger = 0; iFinger < nFingers; iFinger++)
        {

            //
            // extract finger root, finger tip, and finger axis
            //
            EdgeMatcherEdgePair* edgePair = mpEdgeMatcher->edgePair[iFinger];

            cv::Point virtuallStartTip = edgePair->virtualStartTipShortEdge.toCvPoint();
            cv::Point virtuallStartTipScaled = cv::Point((double)virtuallStartTip.x/mScaleFactor, (double)virtuallStartTip.y/mScaleFactor);

            cv::Point virtuallEndTip = edgePair->virtualEndTipShortEdge.toCvPoint();
            cv::Point virtuallEndTipScaled = cv::Point((double)virtuallEndTip.x/mScaleFactor, (double)virtuallEndTip.y/mScaleFactor);

            cv::Point contourStartTip = edgePair->contourStartTip.toCvPoint();
            cv::Point contourStartTipScaled = cv::Point((double)contourStartTip.x/mScaleFactor, (double)contourStartTip.y/mScaleFactor);

            cv::Point contourEndTip = edgePair->contourEndTip.toCvPoint();
            cv::Point contourEndTipScaled = cv::Point((double)contourEndTip.x/mScaleFactor, (double)contourEndTip.y/mScaleFactor);

            cv::Point fingerRoot;
            if (edgePair->virtualStartTipShortEdge.toCvPoint().y < edgePair->virtualEndTipShortEdge.toCvPoint().y)
            {
                fingerRoot = virtuallEndTipScaled;
            }
            else
            {
                fingerRoot = virtuallStartTipScaled;
            }

            cv::Point fingerTip;
            if (edgePair->contourStartTip.toCvPoint().y < edgePair->contourEndTip.toCvPoint().y)
            {
                fingerTip = contourStartTipScaled;
            }
            else
            {
                fingerTip = contourEndTipScaled;
            }

            double axisAngle = std::atan((double)(fingerRoot.y-fingerTip.y)/(double)(fingerRoot.x-fingerTip.x))/CV_PI*180.0;
            if (axisAngle < 0)
            {
                axisAngle += 180.0;
            }




//            //
//            // CHECK: orientation for each finger
//            //
//            if (std::abs(axisAngle) > 90.0+mMaxAngleFinger || std::abs(axisAngle) < 90.0-mMaxAngleFinger)
//            {
//                break;
//            }



            //
            // determine finger width
            //
            int axisPtRef_x = fingerTip.x+cos(axisAngle/180.0*CV_PI)*mOffsetLenForWidth;
            int axisPtRef_y = fingerTip.y+sin(axisAngle/180.0*CV_PI)*mOffsetLenForWidth;
            int xNormal, yNormal;
            double fingerWidthFromAxis1 = 1.0;
            while (1)
            {
                if (fingerWidthFromAxis1 > mMaxIterFingerWidth )
                    break;

                xNormal = axisPtRef_x + fingerWidthFromAxis1 * cos((axisAngle-90.0)/180.0*CV_PI);
                yNormal = axisPtRef_y + fingerWidthFromAxis1 * sin((axisAngle-90.0)/180.0*CV_PI);

                int yNormalScaled = yNormal*mScaleFactor;
                int xNormalScaled = xNormal*mScaleFactor;
                if (yNormalScaled < 0 || xNormalScaled < 0 || yNormalScaled > skinColorMask_ocv.rows-1 || xNormalScaled > skinColorMask_ocv.cols-1)
                    break;
                else if (skinColorMask_ocv.at<unsigned char>(yNormalScaled,xNormalScaled) == 0)
                    break;
                else
                    //skinColorMask_ocv.at<unsigned char>(yNormal*mScaleFactor,xNormal*mScaleFactor) = 128;
                    fingerWidthFromAxis1++;
            }
            double fingerWidthFromAxis2 = 1.0;
            while (1)
            {
                if (fingerWidthFromAxis2 > mMaxIterFingerWidth )
                    break;

                xNormal = axisPtRef_x + fingerWidthFromAxis2 * cos((axisAngle+90.0)/180.0*CV_PI);
                yNormal = axisPtRef_y + fingerWidthFromAxis2 * sin((axisAngle+90.0)/180.0*CV_PI);

                int yNormalScaled = yNormal*mScaleFactor;
                int xNormalScaled = xNormal*mScaleFactor;
                if (yNormalScaled < 0 || xNormalScaled < 0 || yNormalScaled > skinColorMask_ocv.rows-1 || xNormalScaled > skinColorMask_ocv.cols-1)
                    break;
                else if (skinColorMask_ocv.at<unsigned char>(yNormalScaled,xNormalScaled) == 0)
                    break;
                else
                    //skinColorMask_ocv.at<unsigned char>(yNormal*mScaleFactor,xNormal*mScaleFactor) = 128;
                    fingerWidthFromAxis2++;
            }
            double fingerWidth = fingerWidthFromAxis1 + fingerWidthFromAxis2 + 2.0;



            //
            // generate finger object
            //
            SFinger tmp;
            tmp.roiRot.size.height = fingerWidth;//edgePair->getWidth(Vector2D(fingerTip.x, fingerTip.y), 0) * 2.0;
            tmp.roiRot.size.width  = tmp.roiRot.size.height * mRatioFingerHeight2Width;
            tmp.roiRot.center.x = fingerTip.x + tmp.roiRot.size.width/2.0 * cos((axisAngle)/180.0*CV_PI);
            tmp.roiRot.center.y = fingerTip.y + tmp.roiRot.size.width/2.0 * sin((axisAngle)/180.0*CV_PI);
            tmp.roiRot.angle = axisAngle;
            tmp.roi = tmp.roiRot.boundingRect();
            tmp.fingerRoot = fingerRoot;
            tmp.fingerTip = fingerTip;




            //
            //  PLAUSIBILITY CHECKS
            //
            bool validFinger = true;
            //
            // CHECK: orientation for each finger
            //
            if (std::abs(axisAngle) > 90.0+mMaxAngleFinger || std::abs(axisAngle) < 90.0-mMaxAngleFinger)
            {
                validFinger = false;
            }
            //
            // CHECK: finger width
            //
            if (fingerWidth < mMinFingerWidth || fingerWidth > mMaxFingerWidth)
            {
                validFinger = false;
            }
            //
            // CHECK: is ROI within image (for a proper image crop)
            //
            cv::Point2f vertices[4];
            tmp.roiRot.points(vertices);
            for (int i=0; i<4; i++)
            {
                if (vertices[i].x < 0 || vertices[i].x > mInFrameScaled.cols-1 || vertices[i].y < 0 || vertices[i].y > mInFrameScaled.rows-1)
                {
#if 0
                    cout << "i:" << i << " | x:" << vertices[i].x << "/" << mInFrameScaled.cols-1 << " | y:" << vertices[i].y << "/" << mInFrameScaled.cols-1 << std::endl;
#endif
                    validFinger = false;
                }
            }


            if (validFinger)
            {
                //
                // Detected finger seems to be ok -> crop the fingerprint patch from the higher resolution image inFrame
                //
                cv::RotatedRect rect = tmp.roiRot;
                cv::Mat M, rotated, cropped;
                // get angle and size from the bounding box
                float angle = rect.angle;
                cv::Size rect_size = rect.size;
                angle -= 90.0;
                std::swap(rect_size.width, rect_size.height);
                // get the rotation matrix
                M = cv::getRotationMatrix2D(rect.center, angle, 1.0);
                // perform affine transformation
                cv::warpAffine(inFrame, rotated, M, inFrame.size(), cv::INTER_CUBIC);
                // crop the resulting image
                cv::getRectSubPix(rotated, rect_size, rect.center, cropped);
                tmp.fingerPatch = cropped.clone();
                //
                // save current finger to finger list
                //
                fingersPalm.push_back(tmp);
            }
        }

#if 0
        cv::imwrite( QString( QString::number(saveNum) + QString("_0_rgb") + ".png").toStdString(), mInFrameScaled);
        cv::imwrite( QString( QString::number(saveNum) + QString("_1_blob") + ".png").toStdString(), skinColorMask_ocv);
        cvSaveImage( QString( QString::number(saveNum) + QString("_2_contour") + ".png").toLatin1(), mpEdgeMatcher->contourImage);
        cvSaveImage( QString( QString::number(saveNum) + QString("_3_edge") + ".png").toLatin1(), mpEdgeMatcher->edgeImage);
        saveNum++;
#endif

        mpEdgeMatcher->endFrame(); // clean up the memory
        ///
        ///
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    }

    return fingersPalm;
}



 void CFingerprintExtractor::renderResults(cv::Mat& inFrameResult, std::vector<SFinger> fingersPalm)
 {
     for (int iFinger=0; iFinger<fingersPalm.size(); iFinger++)
     {
         cv::line(inFrameResult, fingersPalm.at(iFinger).fingerRoot, fingersPalm.at(iFinger).fingerTip, cv::Scalar(0,255,0), 2);
         cv::circle(inFrameResult, fingersPalm.at(iFinger).fingerRoot, 5, cv::Scalar(0,0,255), -1 );
         cv::circle(inFrameResult, fingersPalm.at(iFinger).fingerTip, 5, cv::Scalar(255,0,0), -1 );

         cv::Point2f vertices[4];
         fingersPalm.at(iFinger).roiRot.points(vertices);
         for (int i = 0; i < 4; i++)
         {
             cv::line(inFrameResult, vertices[i], vertices[(i+1)%4], cv::Scalar(255,255,255), 2);
         }
         //cv::rectangle(inFrameResult, fingersPalm.at(iFinger).roi, cv::Scalar(200,200,200), 3 );
     }

 }
