#include "CEdgeMatcher.h"
#include <list>
#include <fstream>
#include <iostream>
#include <string>
//=================================================================================================
//!
//! Debug & Log Tracing
//!
//=================================================================================================

#define STREAM_LOG      0
#define STREAM_DEBUG    0

#define DEBUG (STREAM_LOG || STREAM_DEBUG)

#if DEBUG
std::ofstream CEdgeMatcher__DebugStream;
//    #define CEdgeMatcher__DebugStream std::cerr;
#endif

#if STREAM_DEBUG
#define DEBUG_STREAM(x) CEdgeMatcher__DebugStream << "[DBG] " << x << endl
#else
#define DEBUG_STREAM(x) //std::cerr << x << std::endl;
#endif

#if STREAM_LOG
#define LOG_STREAM(x) CEdgeMatcher__DebugStream << "[LOG] " << x << endl
#else
#define LOG_STREAM(x)
#endif

//=================================================================================================
//!
//! EdgeMatcherParameters (shared among all edge pair extractors!)
//!
//=================================================================================================

EdgeMatcherParameters* edgeMatcherParameters = NULL;

//=================================================================================================
//!
//! CEdgeMatcher
//!
//=================================================================================================

int CEdgeMatcher::segmentCounter = 0;
int CEdgeMatcher::edgeCounter = 0;

//=================================================================================================
CEdgeMatcher::CEdgeMatcher(Mat& sampleImage)
{
    imageSize = sampleImage.size();

    contourBlobs.create(imageSize, CV_8UC1);
    edgeImage.create(imageSize, CV_8UC3);
    contourImage.create(imageSize, CV_8UC3);
    debugImage.create(imageSize, CV_8UC3);
    borderCrop.create(imageSize, CV_8UC1);

    //contourStorage = cvCreateMemStorage(0);

    if (edgeMatcherParameters == NULL)
    {
        edgeMatcherParameters = new EdgeMatcherParameters();

        edgeMatcherParameters->CONTOUR_MIN_TOTAL_POINTS = 50;
        edgeMatcherParameters->CONTOUR_SMOOTH_POINTS = 10;
        edgeMatcherParameters->CONTOUR_BORDER_CROP = 5;

        edgeMatcherParameters->LINE_MAX_ANGLE_DEVIATION = 20.0;//10.0;
        edgeMatcherParameters->LINE_MAX_ANGLE_DEVIATION_COUNT = 25;
        edgeMatcherParameters->LINE_MIN_LENGTH = 20;
        edgeMatcherParameters->LINE_MAX_CURVINESS = 30;
        edgeMatcherParameters->LINE_MAX_REFLECTION_RATIO = 0.5;

        edgeMatcherParameters->EDGE_MAX_SEGMENT_ANGLE = 20;//12.5;
        edgeMatcherParameters->EDGE_MAX_SEGMENT_DISTANCE_LENGTH_RATIO = 2.0;
        edgeMatcherParameters->EDGE_MAX_SEGMENT_RELATIVE_AXIS_DISTANCE = 10.0; // 1.0
        edgeMatcherParameters->EDGE_MIN_LENGTH = 20;

        edgeMatcherParameters->EDGE_MAX_SEGMENT_ANGLE_DYN =
                EdgeMatcherDynamicThreshold(
                    0.1*imageSize.width, 1.0*edgeMatcherParameters->EDGE_MAX_SEGMENT_ANGLE,
                    0.4*imageSize.width, 0.5*edgeMatcherParameters->EDGE_MAX_SEGMENT_ANGLE);

        edgeMatcherParameters->EDGE_MAX_SEGMENT_DISTANCE_LENGTH_RATIO_DYN =
                EdgeMatcherDynamicThreshold(
                    cos(edgeMatcherParameters->EDGE_MAX_SEGMENT_ANGLE/RAD_TO_DEG), 2.0,
                    1.0, 4.0);

        edgeMatcherParameters->EDGEPAIR_MAX_EDGE_PAIRING_ANGLE = 10.0;//30.0;//80.0;
        edgeMatcherParameters->EDGEPAIR_MIN_EDGE_DISTANCE = 30.0;//10.0;
        edgeMatcherParameters->EDGEPAIR_MIN_BLOB_FILLING = 0.75;
        edgeMatcherParameters->EDGEPAIR_MIN_LENGTH = 40;//80;
    }

    cvInitFont( &font,
                CV_FONT_HERSHEY_SIMPLEX,
                0.4,
                0.4,
                0,
                1,
                CV_AA );

}

//=================================================================================================
CEdgeMatcher::~CEdgeMatcher()
{
    endFrame();

    contourStorage.clear();
    borderCrop.release();
    contourImage.release();
    edgeImage.release();
    contourBlobs.release();
}

//=================================================================================================
EdgeMatcherParameters* CEdgeMatcher::getParameters()
{
    return edgeMatcherParameters;
}

//=================================================================================================
void CEdgeMatcher::limitFrameTo(Mat& mask)
{
    borderCrop = 255;
    rectangle(borderCrop, Point(0, 0), Point(imageSize.width - 1, imageSize.height - 1), Scalar(0));

    if (mask.data )
    {
        bitwise_and(borderCrop, mask, borderCrop);

    }
    erode(borderCrop, borderCrop, getStructuringElement(MORPH_CROSS,Size(3,3)),Point(1,1),
          edgeMatcherParameters->CONTOUR_BORDER_CROP);
}

//=================================================================================================
void CEdgeMatcher::extractEdgePairsFromImage(Mat& mask, const Mat& reflectionsMask, const Mat& bgrFrame /* = NULL */){
    segmentCounter = 1;
    edgeCounter = 1;

#if DEBUG
    CEdgeMatcher__DebugStream = std::ofstream("debug.log", std::ios::out);
#endif

    double tickStart;

    // let contour scanner work on copy since it destroys the image while scanning
    contourBlobs = mask.clone();

    double tickFirst = tickStart = (double)getTickCount();

    // step 1: locate contours within blob image

    findContours(contourBlobs, contourStorage, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

    LOG_STREAM("contour detection: " << ((double)getTickCount() - tickStart)/1000.0 << "ms");
    tickStart = (double)getTickCount();

    // step 2: segment contour hierarchy
    DEBUG_STREAM("### CREATING SEGMENTS #######################");
    //Heirarchy [Next, Previous, First Child, Parent] = [0][0,1,2,3]
    if(contourStorage.size()>0)
    {
        for(int outerIndex = 0; outerIndex != -1; outerIndex = hierarchy[outerIndex][0])
        {
            if (contourStorage[outerIndex].size() < edgeMatcherParameters->CONTOUR_MIN_TOTAL_POINTS)
                continue;

            segmentContour(contourStorage[outerIndex], contourStorage[outerIndex], reflectionsMask);

            for( int innerIndex = hierarchy[outerIndex][2]; innerIndex != -1; innerIndex = hierarchy[innerIndex][0])
            {
                if (contourStorage[outerIndex].size() < edgeMatcherParameters->CONTOUR_MIN_TOTAL_POINTS)
                    continue;
                segmentContour(contourStorage[innerIndex], contourStorage[innerIndex], reflectionsMask);
            }
        }
    }

    LOG_STREAM("segmentation: " << (cvGetTickCount() - tickStart)/1000.0 << "ms");
    tickStart = (double) cvGetTickCount();

    // step 2: combine line segments into edges
    DEBUG_STREAM("### GENERATING EDGES ########################");
    buildEdgesFromLineSegments();

    LOG_STREAM("edge reconstruction: " << (cvGetTickCount() - tickStart)/1000.0 << "ms");
    tickStart = (double) cvGetTickCount();

    // step 3: pair edges by combining "parallel" edges
    DEBUG_STREAM("### PAIRING NON-BORDER EDGES ################");

    pairEdges(false, mask, bgrFrame);

    DEBUG_STREAM("### PAIRING BORDER EDGES ####################");
    //  pairEdges(true, mask, bgrFrame);

    LOG_STREAM("edge pairing: " << (cvGetTickCount() - tickStart)/1000.0 << "ms");
    tickStart = (double) cvGetTickCount();

    // step 4: create edge pair instances from successfully paired edges
    DEBUG_STREAM("### CREATING EDGE PAIRS ##########################");
    //cout << "edge.size() " << edge.size()<< endl;
    for (size_t i = 0; i < edge.size(); i++)
        if ((edge[i]->pairingEdge != NULL) && (edge[i]->assignedTool == NULL) && edge[i]->type != EDGEMATCHER_EDGETYPE_IMGBORDER )
        {
            EdgeMatcherEdgePair * t = new EdgeMatcherEdgePair(edge[i], mask);
            if (t->edgePairingType!=EDGEMATCHER_PAIRED_INVALID) edgePair.push_back(t); else delete t;
        }
    DEBUG_STREAM(edgePair.size() << " edge pairs found");

    LOG_STREAM("edge pair creation: " << (cvGetTickCount() - tickStart)/1000.0 << "ms");
    tickStart = (double) cvGetTickCount();

    // step 5: compute intersections/overlappings of edge pairs
    computeIntersections();

    LOG_STREAM("edge pair extraction done in: " << (cvGetTickCount() - tickFirst)/1000.0 << "ms");

#if DEBUG
    CEdgeMatcher__DebugStream.close();
#endif
}


//=================================================================================================
unsigned int CEdgeMatcher::getBorderType(const Vector2D &rPoint, const Size &rImageSize)
{
    unsigned int borderClass = 0;

    if (rPoint.x <= edgeMatcherParameters->CONTOUR_BORDER_CROP)
        borderClass |= 1;

    if (rPoint.x >= rImageSize.width - edgeMatcherParameters->CONTOUR_BORDER_CROP)
        borderClass |= 2;

    if (rPoint.y <= edgeMatcherParameters->CONTOUR_BORDER_CROP)
        borderClass |= 4;

    if (rPoint.y >= rImageSize.height - edgeMatcherParameters->CONTOUR_BORDER_CROP)
        borderClass |= 8;

    return borderClass;
}



//=================================================================================================
void CEdgeMatcher::renderResults()
{
    contourImage = white;
    edgeImage = white;//Scalar(255,255,255);

    /*if (frame != NULL)
    {
        // draw matched edges
        for (size_t i = 0; i < edgePair.size(); i++)
        {
            EdgeMatcherEdgePair* myTool = edgePair[i];

            // edgePair tip
            //cvCircle(frame, myTool->contourStartTip.toCvPoint(), 3, myTool->rootEdge->colour, 4, 8, 0);
            //cvCircle(frame, myTool->contourEndTip.toCvPoint(), 3, myTool->rootEdge->colour, 4, 8, 0);

            // virtual tip
            cvCircle(frame, myTool->virtualStartTip.toCvPoint(), 3, myTool->rootEdge->colour, -1, 8, 0);
            cvCircle(frame, myTool->virtualEndTip.toCvPoint(), 3, myTool->rootEdge->colour, -1, 8, 0);

            // edgePair axis
            Vector2D pas = myTool->virtualStartTip - 22.0 * myTool->axis;
            Vector2D pae = myTool->virtualEndTip   + 22.0 * myTool->axis;
            cvLine(frame, pas.toCvPoint(), pae.toCvPoint(), myTool->rootEdge->colour, 3, 8, 0);

            // edgePair plain
            //Vector2D pa1 = myTool->axisBase + 5000.0 * myTool->axis;
            //Vector2D pa2 = myTool->axisBase - 5000.0 * myTool->axis;
            //cvLine(frame, pa1.toCvPoint(), pa2.toCvPoint(), myTool->rootEdge->colour, 1, 8, 0);

            // edgePair width in virtual tip
            cvCircle(frame, myTool->virtualStartTip.toCvPoint(), (int)myTool->getWidth(myTool->virtualStartTip, 0), myTool->rootEdge->colour, 2, 8, 0);
            cvCircle(frame, myTool->virtualEndTip.toCvPoint(), (int)myTool->getWidth(myTool->virtualEndTip, 0), myTool->rootEdge->colour, 2, 8, 0);

            //mark "second class" edge pairs
            if (myTool->edgePairingType == EDGEMATCHER_PAIRED_1EDGE_WITHBORDER)
                cvLine(frame, pas.toCvPoint(), pae.toCvPoint(), CV_RGB(55, 55, 55), 1, 8, 0);
            if (myTool->edgePairingType == EDGEMATCHER_PAIRED_WITH_SHORT_BORDER_TOUCHING_EDGES)
                cvLine(frame, pas.toCvPoint(), pae.toCvPoint(), CV_RGB(255, 255, 255), 1, 8, 0);
        }

        // draw edgePair intersections
//        for (size_t i = 0; i < intersection.size(); i++)
//        {
//            cvCircle(frame, intersection[i]->coordinates.toCvPoint(), 20, CV_RGB(255, 255, 255), 4, 8, 0);
//        }
    }*/

    // draw edges
    for (size_t i = 0; i < edge.size(); i++)
    {
        EdgeMatcherEdge* myEdge = edge[i];

        int thickness = 2;//(myEdge->hasRefinement) ? (2) : (1);
        double blendToWhite = myEdge->borderLevel / 5.0;
        Scalar color = CV_RGB(64, 64, 64);

        if (myEdge->assignedTool != NULL)
            color = myEdge->assignedTool->rootEdge->colour;

        for (int j = 0; j < 3; j++)
            color.val[j] = 255*blendToWhite + color.val[j]*(1.0 - blendToWhite);

        line(edgeImage, myEdge->startPoint.toCvPoint(), myEdge->endPoint.toCvPoint(), color, thickness, 8, 0);

        Vector2D center = 0.5*(myEdge->startPoint + myEdge->endPoint);
        line(edgeImage, (center).toCvPoint(), (center + 5 * myEdge->getNormal()).toCvPoint(), color, thickness, 8, 0);

        circle(edgeImage, myEdge->startPoint.toCvPoint(), 3, color, -1, 8, 0);
    }
    // draw contours and segments
    for (size_t i = 0; i < segment.size(); i++)
    {
        EdgeMatcherSegment* mySegment = segment[i];

        for (int i = 0; i < mySegment->points - 1; i++)
            line(contourImage, mySegment->getPoint(i).toCvPoint(), mySegment->getPoint(i + 1).toCvPoint(), CV_RGB(128, 128, 128), 1, 8, 0);

        if (mySegment->hasOrientation)
        {
            line(contourImage, mySegment->stableStartPoint.toCvPoint(), mySegment->stableEndPoint.toCvPoint(), CV_RGB(128, 128, 128), 2, 8, 0);
            circle(contourImage, mySegment->stableStartPoint.toCvPoint(), 5, CV_RGB(64, 64, 64), -1, 8, 0);
        }
    }

#if DEBUG

//    for (size_t i = 0; i < segment.size(); i++)
//    {
//        EdgeMatcherSegment* mySegment = segment[i];
//        char segmentId[50];
//        sprintf_s(segmentId, "[%d]", mySegment->id);
//        Vector2D center = 0.5*(mySegment->getPoint(0) + mySegment->getPoint(-1));
//        center.x = min(center.x, imageSize.width - 20.0);
//        center.y = max(center.y, 12.0);
//        cvPutText(contourImage, segmentId, center.toCvPoint(), &font, white);
//    }

//    for (size_t i = 0; i < edge.size(); i++)
//    {
//        EdgeMatcherEdge* myEdge = edge[i];
//        char edgeId[50];
//        sprintf_s(edgeId, "[%d]", myEdge->id);
//        Vector2D center = 0.5*(myEdge->startPoint + myEdge->endPoint);
//        center.x = min(center.x, imageSize.width - 20.0);
//        center.y = max(center.y, 12.0);
//        cvPutText(edgeImage, edgeId, center.toCvPoint(), &font, white);
//    }

#endif

}

//=================================================================================================
void CEdgeMatcher::endFrame()
{
    for (size_t i = 0; i < intersection.size(); i++)
    {
        delete intersection[i];
    }

    intersection.clear();

    for (size_t i = 0; i < edgePair.size(); i++)
    {
        delete edgePair[i];
    }

    edgePair.clear();

    for (size_t i = 0; i < edge.size(); i++)
    {
        delete edge[i];
    }
    edge.clear();

    for (size_t i = 0; i < segment.size(); i++)
    {
        delete segment[i];
    }

    segment.clear();

    for (size_t i = 0; i < contour.size(); i++)
        contour[i].clear();

    contour.clear();
    contourStorage.clear();
}


//=================================================================================================
void CEdgeMatcher::computeIntersections()
{
    for (size_t i = 0; i < edgePair.size(); i++)
    {
        EdgeMatcherEdgePair* edgePairA = edgePair[i];

        for (size_t j = i + 1; j < edgePair.size(); j++)
        {
            EdgeMatcherEdgePair* edgePairB = edgePair[j];

            // compute intersection of the edgePair axes
            double l1 = (edgePairA->axisBase - edgePairB->axisBase) * edgePairA->axisNormal;
            double l2 = edgePairA->axisNormal * edgePairB->axisNormal.determineNormal();;

            if (fabs(l2) > 1e-10)
            {
                double l = l1 / l2;

                Vector2D coordinates = edgePairB->axisBase + l * edgePairB->axisNormal.determineNormal();

                if ((edgePairA->pointIsBetweenTips(coordinates)) && (edgePairB->pointIsBetweenTips(coordinates)))
                {
                    intersection.push_back(new EdgeMatcherIntersection(edgePairA, edgePairB, coordinates));
                }
            }
        }
    }
}


//=================================================================================================
void CEdgeMatcher::pairEdges(bool pairWithBorders, const Mat& blobImage /* = NULL */, const Mat& bgrFrame /* = NULL */)
{
    ToolExtractorEdgeSort sort;
    std::sort(edge.begin(), edge.end(), sort);

    for (int i = 0; i < ((int) edge.size()) - 1; i++)
    {
        EdgeMatcherEdge* a = edge[i];
        if (a->type == EDGEMATCHER_EDGETYPE_IMGBORDER)
            continue;

        EdgeMatcherEdge* b = a->pairingEdge;
        if (pairWithBorders && b) continue;

        double quality = a->pairingEdgeQuality;

        for (size_t j = pairWithBorders ? 0 : i+1; j < edge.size(); j++)
        {
            EdgeMatcherEdge* testB = edge[j];
            if ((!pairWithBorders && testB->type == EDGEMATCHER_EDGETYPE_IMGBORDER) ||
                    (pairWithBorders && testB->type != EDGEMATCHER_EDGETYPE_IMGBORDER))
                continue;

            double qualityTest = a->testEdgeAsPair(testB, blobImage, bgrFrame);

            if (quality < qualityTest)
            {
                b = testB;
                quality = qualityTest;
            }
        }

        if (b != a->pairingEdge)
        {
            bool pairEdges = quality>0.0;

            if (b->pairingEdge != NULL)
            {
                // b already is paired with an edge -> remove pairing only when quality is improved!
                if (quality > b->pairingEdgeQuality)
                {
                    b->pairingEdge->unpairEdge();
                }
                else
                {
                    pairEdges = false;
                }
            }

            if (pairEdges)
            {
                if (a->pairingEdge != NULL)
                {
                    a->pairingEdge->unpairEdge();
                }
                a->pairEdgeWith(b, quality);
            }
        }
    }
}


//=================================================================================================
void CEdgeMatcher::buildEdgesFromLineSegments()
{
    // create list of line segments that will be used for edge reconstruction
    std::vector<EdgeMatcherSegment*> segmentBySize;

    for (size_t i = 0; i < segment.size(); i++)
    {
        if (segment[i]->hasOrientation)
        {
            segmentBySize.push_back(segment[i]);
        }
    }

    if (segmentBySize.size() == 0)
    {
        return;
    }
    // combine segments into edges, starting with largest ones
    ToolExtractorSegmentSort sort;
    std::sort(segmentBySize.begin(), segmentBySize.end(), sort);

    for (size_t i = 0; i < segmentBySize.size(); i++)
    {
        if (segmentBySize[i]->assignedEdge != NULL)
            continue;

        EdgeMatcherEdge* newEdge = new EdgeMatcherEdge(segmentBySize[i]);
        //cout << "i : " << i <<  " line : " << __LINE__<<endl;
        if (newEdge->type == EDGEMATCHER_EDGETYPE_EDGEPAIR)
        {
            while (1)
            {
                bool hasRefinement = false;

                for (size_t j = 0; j < segmentBySize.size(); j++)
                {
                    if (j == i)
                        continue;

                    if (segmentBySize[j]->onBorder == LIES_ON_BORDER)
                        continue;

                    if (segmentBySize[j]->assignedEdge != NULL)
                        continue;
                    if ((segmentBySize[i]->parentContour == segmentBySize[j]->parentContour))
                    {
                        hasRefinement |= newEdge->refineUsingSegment(segmentBySize[j]);
                    }
                }

                if (!hasRefinement)
                    break;
            }
        }
        if (newEdge->quality > 0.0)
        {
            edge.push_back(newEdge);
        }
        else
            delete newEdge;
    }
}

//=================================================================================================
bool CEdgeMatcher::isFarFromImageBorder(Vector2D point)
{
    return ( borderCrop.at<uchar>((int)(point.y + 0.5)) * borderCrop.cols , ((int)(point.x + 0.5)) != 0);
}

#define LOOKUP_WRAP(a,b) ((a + 2 * b) % b)
#define LOOKUP(a,b,c) (a[LOOKUP_WRAP(b, c)])
//=================================================================================================
void CEdgeMatcher::segmentContour(vector<Point> contour, vector<Point> parentContour, const Mat& reflectionsMask)
{
    // step 1: compute tangent angle for all contour pixels by averaging
    // a couple of pixels before and after a center pixel
    EdgeMatcherContourPoint *pContourPoint = new EdgeMatcherContourPoint[contour.size()];

    for (int i = 0; i < contour.size(); i++)
    {
        pContourPoint[i].setBase(i, contour[i]);
    }
    Vector2D pixelSummAhead(0.0, 0.0);
    Vector2D pixelSummBehind(0.0, 0.0);

    for (int i = 0; i < edgeMatcherParameters->CONTOUR_SMOOTH_POINTS; i++)
    {
        pixelSummAhead  += pContourPoint[i].coord;
        pixelSummBehind += pContourPoint[contour.size() - 1 - i].coord;
    }
    for (int i = 0; i < contour.size(); i++)
    {
        Vector2D &rvAhead  = LOOKUP(pContourPoint, i + edgeMatcherParameters->CONTOUR_SMOOTH_POINTS, contour.size()).coord;
        Vector2D &rvCenter = LOOKUP(pContourPoint, i, contour.size()).coord;
        Vector2D &rvBehind = LOOKUP(pContourPoint, i - edgeMatcherParameters->CONTOUR_SMOOTH_POINTS, contour.size()).coord;

        pixelSummAhead  += rvAhead - rvCenter;

        Vector2D vectorDiffAhead = pixelSummAhead/(double)edgeMatcherParameters->CONTOUR_SMOOTH_POINTS - rvCenter;
        Vector2D vectorDiffBehind = rvCenter - pixelSummBehind/(double)edgeMatcherParameters->CONTOUR_SMOOTH_POINTS;

        pixelSummBehind += rvCenter - rvBehind;

        pContourPoint[i].setAngle(vectorDiffAhead, vectorDiffBehind);
    }

    // step 2: create seed points for line segments by sorting contour
    // points by increasing difference of 'before' and 'after' tangent angles
    std::vector< EdgeMatcherContourPoint *> vSeedPoints;
    bool useGoodPointAsStart = true;
    bool useBadPointAsEnd = false;
    int startIndex = -1;

    for (int i = 0; i < contour.size(); i++)
    {
        // there typically are more than one consecutive seed points
        // on a contour. for keeping the number of seed points low,
        // we only take one (i.e. the center) of a list of consecutive
        // seed points.
        if (pContourPoint[i].angleDifference < 0.5*edgeMatcherParameters->LINE_MAX_ANGLE_DEVIATION)
        {
            if (useGoodPointAsStart)
            {
                startIndex = i;

                useGoodPointAsStart = false;
                useBadPointAsEnd = true;
            }
        }
        else
        {
            if (useBadPointAsEnd)
            {
                vSeedPoints.push_back(&pContourPoint[(startIndex + i)/2]);

                useGoodPointAsStart = true;
                useBadPointAsEnd = false;
            }
        }
    }

    if (useBadPointAsEnd)
    {
        // we were in a seed point segment that ended with the contour itself
        vSeedPoints.push_back(&pContourPoint[(startIndex + contour.size())/2]);
    }

    std::sort(vSeedPoints.begin(), vSeedPoints.end(), EdgeMatcherContourPoint::compareUsingAngle);

    // step 3: locate line segments
    for (size_t i = 0; i < vSeedPoints.size(); i++)
    {
        if (vSeedPoints[i]->declaredAsLine)
            continue;

        generateLineSegment(vSeedPoints[i], pContourPoint, contour, parentContour, reflectionsMask);
    }

    // step 4: clean-up
    delete [] pContourPoint;

    DEBUG_STREAM("----------------------");
}


//=================================================================================================
void CEdgeMatcher::generateLineSegment(EdgeMatcherContourPoint *pSeedPoint, EdgeMatcherContourPoint *pContourPoint,
                                       vector<Point> pOcvContour, vector<Point> pOcvParentContour, const Mat& reflectionsMask)
{
    // we start with a point having almost the same angle ahead as behind
    pSeedPoint->declaredAsLine = true;

    // check extent of potential line segment starting from seed point
    int iMoveForward = displaceSegmentEndPoint(pSeedPoint, pContourPoint, (int)pOcvContour.size(), true);
    int iMoveBackward = displaceSegmentEndPoint(pSeedPoint, pContourPoint, (int)pOcvContour.size(), false);
    int iPointCount = 1 + iMoveForward - iMoveBackward;
    if (iPointCount < edgeMatcherParameters->LINE_MIN_LENGTH)
        return;

    // check curviness of potential line segment
    EdgeMatcherContourPoint &rStartPoint = LOOKUP(pContourPoint, pSeedPoint->index + iMoveBackward, pOcvContour.size());
    EdgeMatcherContourPoint *pPreviousPoint = &rStartPoint;
    double dAveragedAngleDifference = 0.0;

    double nReflections = 0;
    for (int i = 1; i <= iPointCount; i++)
    {
        EdgeMatcherContourPoint &rPoint = LOOKUP(pContourPoint, pSeedPoint->index + iMoveBackward + i, pOcvContour.size());

        dAveragedAngleDifference += fabs(ANGLE_DIFFERENCE_DEG(rPoint.angle, pPreviousPoint->angle));

        if (!reflectionsMask.empty())
            nReflections += (( reflectionsMask.at<uchar>((int) rPoint.coord.y)*reflectionsMask.cols,(int) rPoint.coord.x )!=0);

        pPreviousPoint = &rPoint;
    }
    dAveragedAngleDifference /= (double) iPointCount;
    if (dAveragedAngleDifference > edgeMatcherParameters->LINE_MAX_CURVINESS)
        return;

    nReflections /= (iPointCount+1);
    if (nReflections > edgeMatcherParameters->LINE_MAX_REFLECTION_RATIO)
        return;
    // we have a new segment!
    EdgeMatcherSegment *pNewSegment = new EdgeMatcherSegment(pOcvContour, pOcvParentContour, rStartPoint.index, iPointCount, imageSize);
    pNewSegment->linearity = 1.0 - dAveragedAngleDifference/edgeMatcherParameters->LINE_MAX_CURVINESS;

    DEBUG_STREAM("segment " << pNewSegment->id << ":");
    DEBUG_STREAM("from (" << pNewSegment->getPoint( 0).x << ", " << pNewSegment->getPoint( 0).y << "), i.e. index " << LOOKUP_WRAP(pSeedPoint->index + iMoveBackward, pOcvContour.size()));
    DEBUG_STREAM("to   (" << pNewSegment->getPoint(-1).x << ", " << pNewSegment->getPoint(-1).y << "), i.e. index " << LOOKUP_WRAP(pSeedPoint->index + iMoveForward, pOcvContour.size()));
    DEBUG_STREAM("linearity = " << pNewSegment->linearity);
    DEBUG_STREAM("---");

    segment.push_back(pNewSegment);
}


//=================================================================================================
int CEdgeMatcher::displaceSegmentEndPoint(EdgeMatcherContourPoint *pSeedPoint, EdgeMatcherContourPoint *pContourPoint, int iContourPoints, bool bForward)
{
    int iDirection = (bForward) ? (1) : (-1);
    int iDisplacement = 1;
    int iErrorPixels = 0;
    int iMovement = 0;

    while (iDisplacement < iContourPoints)
    {
        EdgeMatcherContourPoint &rPotentialEndPoint = LOOKUP(pContourPoint, pSeedPoint->index + iDirection*iDisplacement, iContourPoints);

        if (rPotentialEndPoint.declaredAsLine)
            break;

        bool bAcceptNewEndPoint = false;

        if (ANGLE_DIFFERENCE_DEG(rPotentialEndPoint.getAngle(bForward), pSeedPoint->getAngle(bForward)) <= edgeMatcherParameters->LINE_MAX_ANGLE_DEVIATION)
        {
            // when we're back from a couple of curved pixels, we should check
            // whether we're still "close" to the inital seedpoint's hyperplane
            if (iErrorPixels > 0.5*edgeMatcherParameters->LINE_MAX_ANGLE_DEVIATION_COUNT)
                bAcceptNewEndPoint = (fabs((rPotentialEndPoint.coord - pSeedPoint->coord)*pSeedPoint->normal) < 5.0);
            else
                bAcceptNewEndPoint = true;
        }

        if (bAcceptNewEndPoint)
        {
            do
            {
                iMovement++;
                LOOKUP(pContourPoint, pSeedPoint->index + iDirection*iMovement, iContourPoints).declaredAsLine = true;
            } while (iMovement < iDisplacement);

            iErrorPixels = 0;
        }
        else
        {
            iErrorPixels++;

            if (iErrorPixels > edgeMatcherParameters->LINE_MAX_ANGLE_DEVIATION_COUNT)
                break;
        }

        iDisplacement++;
    }

    return iDirection*iMovement;
}

//=================================================================================================
//!
//! EdgeMatcherSegment
//!
//=================================================================================================

//=================================================================================================
EdgeMatcherSegment::EdgeMatcherSegment()
{
    assignedEdge = NULL;
    contour.clear();
    hasOrientation = false;
}


//=================================================================================================
EdgeMatcherSegment::EdgeMatcherSegment(vector<Point> _contour, vector<Point> _parentContour, int _startPoint, int _points, Size &_imageSize)
    : id(CEdgeMatcher::segmentCounter++)
    , parentContour(_parentContour)
    , contour(_contour)
    , startPoint(_startPoint)
    , endPoint((_startPoint + _points) % (int)_contour.size())
    , points(_points)
    , assignedEdge(NULL)
    , hasOrientation(false)
    , linearity(0.0)
{
    Vector2D end = getPoint(-1);
    // characterize segment-type
    unsigned int borderStart = CEdgeMatcher::getBorderType(getPoint(0), _imageSize);
    unsigned int borderEnd = CEdgeMatcher::getBorderType(getPoint(-1), _imageSize);

    if ((borderStart*borderEnd)!=0)
    {
        if (borderStart == borderEnd) onBorder = LIES_ON_BORDER;
        else onBorder = CUTS_FRAME_CORNER;
    }
    else
        if ((borderStart + borderEnd) == 0) onBorder = TOUCHES_NO_BORDER;
        else onBorder = TOUCHES_ONE_BORDER;

    // compute orientation if line is long enough
    if (points < edgeMatcherParameters->LINE_MIN_LENGTH)
    {
        return;
    }

    Vector2D a(0.0, 0.0);
    Vector2D b(0.0, 0.0);

    // average a couple of points on the left and right half of line segment
    for (int i = 0; i < edgeMatcherParameters->CONTOUR_SMOOTH_POINTS / 2; i++)
    {
        int j = points * i / edgeMatcherParameters->CONTOUR_SMOOTH_POINTS;

        a += getPoint(j);
        b += getPoint(-j - 1);
    }

    a /= (edgeMatcherParameters->CONTOUR_SMOOTH_POINTS / 2);
    b /= (edgeMatcherParameters->CONTOUR_SMOOTH_POINTS / 2);

    updateNormal(a, b);
}


//=================================================================================================
inline Vector2D EdgeMatcherSegment::getPoint(int index)
{
    while (index < 0)
    {
        index += points;
    }

    while (index >= points)
    {
        index -= points;
    }

    return Vector2D( contour[ (startPoint + index)%contour.size()]);
}


//=================================================================================================
double EdgeMatcherSegment::getDistanceFromPoint(Vector2D* point)
{
    return (*point - stableStartPoint) * normal;
}

double EdgeMatcherSegment::getLength()
{
    return (getPoint(0) - getPoint(-1)).getLength();
}


//=================================================================================================
void EdgeMatcherSegment::updateNormal(Vector2D a, Vector2D b)
{
    stableStartPoint = a;
    stableEndPoint = b;
    stablePointDistance = stableStartPoint.getDistanceTo(stableEndPoint);

    Vector2D newNormal = (stableEndPoint - stableStartPoint).determineNormal();

    if ((hasOrientation) && (normal * newNormal < 0.0))
    {
        normal = -1.0 * newNormal;
    }
    else
    {
        normal = newNormal;
    }

    hasOrientation = true;
}

//=================================================================================================
//!
//! EdgeMatcherEdge
//!
//=================================================================================================

//=================================================================================================
EdgeMatcherEdge::EdgeMatcherEdge()
{
}

//=================================================================================================
EdgeMatcherEdge::EdgeMatcherEdge(EdgeMatcherSegment* _rootSegment)
{
    id = CEdgeMatcher::edgeCounter++;
    rootSegment = _rootSegment;
    rootSegment->assignedEdge = this;

    matchingSegment = *rootSegment; // copy this one since we'll tweak it!

    assignedTool = NULL;

    edgeDrawn = false;

    startPoint = rootSegment->getPoint(0);
    endPoint = rootSegment->getPoint(-1);
    length = startPoint.getDistanceTo(endPoint);

    borderLevel = _rootSegment->onBorder;

    // edge type depends on its border type
    type = (borderLevel == LIES_ON_BORDER) ? (EDGEMATCHER_EDGETYPE_IMGBORDER) : (EDGEMATCHER_EDGETYPE_EDGEPAIR);

    // edge quality (NOT pairing quality) depends on its length and border type
    if (length > edgeMatcherParameters->EDGE_MIN_LENGTH)
        quality = 1.0;
    else if (length > edgeMatcherParameters->EDGE_MIN_LENGTH/2 && borderLevel == TOUCHES_ONE_BORDER)
        quality = 0.5;
    else
        quality = 0.0;

    hasRefinement = false;

    unpairEdge();
}


//=================================================================================================
double EdgeMatcherEdge::getDistanceFromPoint(Vector2D* point)
{
    return (*point - 0.5*(startPoint + endPoint)) * getNormal();
}


//=================================================================================================
void EdgeMatcherEdge::pairEdgeWith(EdgeMatcherEdge* edge, double quality)
{
    pairingEdge = edge;
    pairingEdgeQuality = quality;

    edge->pairingEdge = this;
    edge->pairingEdgeQuality = quality;
    edge->colour = colour;
}


//=================================================================================================
void EdgeMatcherEdge::unpairEdge()
{
    pairingEdge = NULL;
    pairingEdgeQuality = 0.0;

    Vector2D col = Vector2D(rand() & 255, rand() & 255).normalize();
    int si = rand() % 3;

    colour.val[(si + 0) % 3] = 255.0*col.x;
    colour.val[(si + 1) % 3] = 255.0*col.y;
    colour.val[(si + 2) % 3] = 255.0*col.y;
    colour.val[           3] = 0;
}


//=================================================================================================
double EdgeMatcherEdge::testEdgeAsPair(EdgeMatcherEdge* edge, const Mat& blobImage /* = NULL */, const Mat& bgrFrame /* = NULL */)
{
    DEBUG_STREAM("======== " << id << "#" << edge->id << " =================");

    if ((borderLevel > 1) && (edge->borderLevel > 1))
    {
        DEBUG_STREAM("Has border flag for both edges! That's bad!");
        return 0.0;
    }

    double cosNormal = getNormal() * edge->getNormal();
    double maxCos = cos(edgeMatcherParameters->EDGEPAIR_MAX_EDGE_PAIRING_ANGLE / RAD_TO_DEG);

    // edge isn't valid pair when angle between normals is too far from 180deg
    DEBUG_STREAM("Angle difference: " << cosNormal << " !< " << -maxCos << " (" << edgeMatcherParameters->EDGEPAIR_MAX_EDGE_PAIRING_ANGLE << ")");
    if (cosNormal > -maxCos)
    {
        DEBUG_STREAM("That's bad!");
        return 0.0;
    }

    // edge is valid only when start and end point are on negative side
    double tes = getDistanceFromPoint(&edge->startPoint);
    double tee = getDistanceFromPoint(&edge->endPoint);
    if ((tes > 4) || (tee > 4))
    {
        DEBUG_STREAM("End points of B not both on negative side of A (" << tes << ", " << tee << ")");
        return 0.0;
    }

    double res = edge->getDistanceFromPoint(&startPoint);
    double ree = edge->getDistanceFromPoint(&endPoint);
    if ((res > 4) || (ree > 4))
    {
        DEBUG_STREAM("End points of A not both on negative side of B (" << res << ", " << ree << ")");
        return 0.0;
    }

    Vector2D centerA = 0.5*(startPoint + endPoint);
    Vector2D centerB = 0.5*(edge->startPoint + edge->endPoint);
    double centerDistance = centerA.getDistanceTo(centerB);

    // edge is valid only when the distance between to edges is > EDGEMATCHER_MIN_EDGE_DISTANCE
    DEBUG_STREAM("Distance is: " << centerDistance << " (MIN=" << edgeMatcherParameters->EDGEPAIR_MIN_EDGE_DISTANCE << ", MAX=" << length << ")");

    if (centerDistance <  edgeMatcherParameters->EDGEPAIR_MIN_EDGE_DISTANCE)
    {
        DEBUG_STREAM("That's too close!");
        return 0.0;
    }

    // edge is valid only when the distance between two edges is < the edge's length
    // this restriction does not apply to short edge pairs close to border
    //if (borderLevel == TOUCHES_NO_BORDER || edge->borderLevel == TOUCHES_NO_BORDER)
    if (centerDistance > length)
    {
        DEBUG_STREAM("That's too far!");
        return 0.0;
    }

    // do a first metric check against boundaries here since the remaining metrics
    // are expensive to compute

    double angleMatch = -(cosNormal + maxCos) / (1 - maxCos);
    angleMatch = 0.5 + 0.5*angleMatch;  // a suggestion to penalize perspective-deformed edge pairs less
    DEBUG_STREAM("angleMatch = " << angleMatch);

    double lengthRatio = (length < edge->length) ? (length / edge->length) : (edge->length / length);
    DEBUG_STREAM("lengthRatio = " << lengthRatio);

    double relativeDistance = centerDistance / max(length, edge->length);
    DEBUG_STREAM("relativeDistance = " << relativeDistance);

    double linearity = (matchingSegment.linearity*matchingSegment.stablePointDistance + edge->matchingSegment.linearity*edge->matchingSegment.stablePointDistance)/(matchingSegment.stablePointDistance + edge->matchingSegment.stablePointDistance);
    DEBUG_STREAM("linearity = " << linearity);

    double pairQuality = lengthRatio * angleMatch / relativeDistance * linearity; // * this->quality * edge->quality;
    DEBUG_STREAM("pairQuality = " << pairQuality);

    if (pairQuality < 0.25)
        return 0.0;

    // determine shape of potential edgePair
    Vector2D axisBase;
    Vector2D axis;
    EdgeMatcherEdgePair::computeAxis(this, edge, axisBase, axis);

    Vector2D startTip;
    Vector2D endTip;
    EdgeMatcherEdgePair::computeTips(this, edge, axisBase, axis, startTip, endTip,axis,axisBase);

    std::vector< Vector2D > polyPoint;
    EdgeMatcherEdgePair::computeShape(this, edge, startTip, endTip, polyPoint);
    polyPoint.push_back(polyPoint[0]);

    std::vector< Vector2D > polySample;
    if (!blobImage.empty())
    {
        //generateShapeSamples(polyPoint,blobImage.size() , polySample);
    }
    int polySamples = int(polySample.size());

    // computationally expensive: "check" blob gaps between edges
    double blobFilling = 1.0;

    if (!blobImage.empty())
    {
        blobFilling = 0.0;

        for (size_t i = 0; i < polySamples; i++)
        {
            Vector2D &p = polySample[i];

            if (blobImage.at<uchar>(int(p.x + 0.5) + blobImage.cols*int(p.y + 0.5)) != 0)
                blobFilling += 1.0;
        }

        blobFilling /= max(1, polySamples);
    }
    DEBUG_STREAM("blobFilling = " << blobFilling << " (" << polySample.size() << " samples)");

    if (blobFilling < edgeMatcherParameters->EDGEPAIR_MIN_BLOB_FILLING)
    {
        DEBUG_STREAM("That's bad! (!> " << edgeMatcherParameters->EDGEPAIR_MIN_BLOB_FILLING << ")");
        return 0.0;
    }

    pairQuality *= blobFilling;
    DEBUG_STREAM("pairQuality is now = " << pairQuality);

    if (pairQuality < 0.25)
        return 0.0;

    // computationally expensive: "check" colour "deviation" within blob
    /*double colourDeviation = 1.0;

    if (bgrFrame != NULL)
    {
        colourDeviation = 0.0;

        double colMeanR = 0.0;
        double colMeanG = 0.0;
        double colMeanB = 0.0;

        for (size_t i = 0; i < polySamples; i++)
        {
            Vector2D &p = polySample[i];
            unsigned char *pixel = (unsigned char *) bgrFrame->imageData + int(p.y)*bgrFrame->widthStep + int(p.x)*3;

            colMeanB += pixel[0];
            colMeanG += pixel[1];
            colMeanR += pixel[2];
        }

        double colMean = sqrt(colMeanB*colMeanB + colMeanG*colMeanG + colMeanR*colMeanR);

        colMeanB /= colMean;
        colMeanG /= colMean;
        colMeanR /= colMean;

        for (size_t i = 0; i < polySamples; i++)
        {
            Vector2D &p = polySample[i];
            unsigned char *pixel = (unsigned char *) bgrFrame->imageData + int(p.y)*bgrFrame->widthStep + int(p.x)*3;

            double colB = pixel[0];
            double colG = pixel[1];
            double colR = pixel[2];

            double col = sqrt(colB*colB + colG*colG + colR*colR);

            colB /= col;
            colG /= col;
            colR /= col;

            double dev = colB*colMeanB + colG*colMeanG + colR*colMeanR;
            double ang = acos(dev);
            double dif = 1.0 - 2.0*ang/CV_PI;

            colourDeviation += dif;
        }

        colourDeviation /= max(1, polySamples);

    }

    DEBUG_STREAM("colourDeviation = " << colourDeviation << " (" << polySample.size() << " samples)");

    pairQuality *= colourDeviation;
    DEBUG_STREAM("pairQuality is now = " << pairQuality);

    if (pairQuality < 0.25)
        return 0.0;
    */

    // computationally expensive: "check" gradient within rgb frame
    /*const int gradientLines = 3;
    const int gradientSamples = 100;
    double gradientValues[gradientLines] = { 0 };
    double gradientMinVal = DBL_MAX;
    double gradientMaxVal = DBL_MIN;
    unsigned char *imageData = (unsigned char *) bgrFrame->imageData;

    for (int i = 0; i < gradientLines; i++)
    {
        double blendLine = 0.25 + 0.5 * i / (gradientLines - 1);
        Vector2D gradientStart = startPoint*blendLine + edge->startPoint*(1.0 - blendLine);
        Vector2D gradientEnd = endPoint*blendLine + edge->endPoint*(1.0 - blendLine);

        gradientValues[i] = 0.0;

        for (int j = 0; j < gradientSamples; j++)
        {
            double blendSample = 0.25 + 0.5 * j / (gradientSamples - 1);
            Vector2D gradientPos = gradientStart*blendSample + gradientEnd*(1.0 - blendSample);
            int pixelOffset = int(gradientPos.x)*3 + int(gradientPos.y)*bgrFrame->widthStep;

            gradientValues[i] +=
                    0.1*imageData[pixelOffset    ] +
                    0.6*imageData[pixelOffset + 1] +
                    0.3*imageData[pixelOffset + 2];
        }

        gradientValues[i] /= gradientSamples;

        gradientMinVal = min(gradientMinVal, gradientValues[i]);
        gradientMaxVal = max(gradientMaxVal, gradientValues[i]);

        DEBUG_STREAM("gradientValues[" << i << "] = " << gradientValues[i]);
    }

    double gradientExtent = gradientMaxVal - gradientMinVal;
    double gradientMetricWeight = sqrt(gradientExtent/255.0);
    double relativeCenterGradient = (gradientValues[(gradientLines - 1)/2] - gradientMinVal)/gradientExtent;
    double centerGradientMetric = 1.0*(1.0 - gradientMetricWeight) + sqrt(relativeCenterGradient)*gradientMetricWeight;

    DEBUG_STREAM(
                "relativeCenterGradient = " << relativeCenterGradient << ", " <<
                "gradientMetricWeight = " << gradientMetricWeight << " => " <<
                "centerGradientMetric = " << centerGradientMetric);

    // compute final pair quality
    pairQuality *= centerGradientMetric;
    DEBUG_STREAM("pairQuality is now = " << pairQuality);

    if (pairQuality < 0.25)
        return 0.0;
    */
    //cout << "pairQuality " <<pairQuality << endl;
    return pairQuality;
}


//=================================================================================================
void EdgeMatcherEdge::generateShapeSamples(std::vector< Vector2D > &rvPolyPoint, Size &rImageSize, std::vector< Vector2D > &rvSamplePoint)
{
    int iCount = (int) rvPolyPoint.size();

    if (iCount < 3)
        return;

    std::vector< Vector2D > polyNormal;
    polyNormal.push_back((rvPolyPoint[1] - rvPolyPoint[0]).determineNormal());

    int polyMinX = rvPolyPoint[0].x;
    int polyMaxX = rvPolyPoint[0].x;
    int polyMinY = rvPolyPoint[0].y;
    int polyMaxY = rvPolyPoint[0].y;

    for (int i = 1; i < iCount - 1; i++)
    {
        polyNormal.push_back((rvPolyPoint[i + 1] - rvPolyPoint[i]).determineNormal());

        polyMinX = min(polyMinX, int(rvPolyPoint[i].x));
        polyMaxX = max(polyMaxX, int(rvPolyPoint[i].x));
        polyMinY = min(polyMinY, int(rvPolyPoint[i].y));
        polyMaxY = max(polyMaxY, int(rvPolyPoint[i].y));
    }

    polyMinX = max(polyMinX, 0);
    polyMaxX = min(polyMaxX, rImageSize.width - 1);
    polyMinY = max(polyMinY, 0);
    polyMaxY = min(polyMaxY, rImageSize.height - 1);

    const int iStep = 2;

    for (int y = polyMinY; y <= polyMaxY; y += iStep)
    {
        for (int x = polyMinX; x <= polyMaxX; x += iStep)
        {
            Vector2D p(x, y);
            bool abort = false;

            for (int i = 0; i < iCount - 1; i++)
            {
                Vector2D diff = p - rvPolyPoint[i];
                double mul = diff*polyNormal[i];

                if (mul < 2)
                {
                    abort = true;
                    break;
                }
            }

            if (abort)
                continue;

            rvSamplePoint.push_back(p);
        }
    }
}


//=================================================================================================
Vector2D EdgeMatcherEdge::getCenter()
{
    return 0.5 * (matchingSegment.stableStartPoint + matchingSegment.stableEndPoint);
}


//=================================================================================================
Vector2D EdgeMatcherEdge::getNormal()
{
    return matchingSegment.normal;
}


//=================================================================================================
void EdgeMatcherEdge::getEndpointMetrics(EdgeMatcherSegment* segment, double &rDistance, double &rAngle)
{
    Vector2D v[4] = {
        segment->getPoint( 0) - startPoint,
        segment->getPoint( 0) - endPoint,
        segment->getPoint(-1) - startPoint,
        segment->getPoint(-1) - endPoint };

    Vector2D best = v[0];

    for (int i = 1; i < 4; i++)
        if (best.getLength() > v[i].getLength())
            best = v[i];

    rDistance = best.getLength();
    rAngle = atan2(best.y, best.x)*RAD_TO_DEG;
}


//=================================================================================================
bool EdgeMatcherEdge::refineUsingSegment(EdgeMatcherSegment* segment)
{
    DEBUG_STREAM("======== " << rootSegment->id << ":" << segment->id << " =================");

    DEBUG_STREAM("Border flags are: " << matchingSegment.onBorder << " / " << segment->onBorder);

    // check whether angle between normals is too large
    double cosNormal = getNormal() * segment->normal;
    double cosNormalDynThresAngle = edgeMatcherParameters->EDGE_MAX_SEGMENT_ANGLE_DYN(length);
    double cosNormalDynThres = cos(cosNormalDynThresAngle / RAD_TO_DEG);

    DEBUG_STREAM("Angle: " << cosNormal << " !> " << cosNormalDynThres << " (" << cosNormalDynThresAngle << "�)");

    if (cosNormal < cosNormalDynThres)
    {
        DEBUG_STREAM("That's bad!");
        return false;
    }

    // check whether "endpoint distance : min segment length" ratio is too large
    double endPointDistance;
    double endPointAngle;
    getEndpointMetrics(segment, endPointDistance, endPointAngle);

    double minLength = min(matchingSegment.getLength(), segment->getLength());
    double ratioEpdMl = endPointDistance/minLength;
    double ratioEpdMlDynThresh = edgeMatcherParameters->EDGE_MAX_SEGMENT_DISTANCE_LENGTH_RATIO_DYN(cosNormal);

    DEBUG_STREAM("Ratio of EndPointDistance/MinLength: " << endPointDistance << " / " << minLength << " = " << ratioEpdMl << " <! " << ratioEpdMlDynThresh);
    if (ratioEpdMl > ratioEpdMlDynThresh)
    {
        DEBUG_STREAM("That's bad!");
        return false;
    }

    // check whether scaled distance of endpoints to edge axis is too large
    Vector2D remoteStartPoint = segment->getPoint(0);
    Vector2D remoteEndPoint = segment->getPoint(-1);

    double d1 = fabs(matchingSegment.getDistanceFromPoint(&remoteStartPoint));
    double d2 = fabs(matchingSegment.getDistanceFromPoint(&remoteEndPoint));

    double distanceScale = endPointDistance/100.0;
    double maxDist = max(10.0, distanceScale*edgeMatcherParameters->EDGE_MAX_SEGMENT_RELATIVE_AXIS_DISTANCE);

    DEBUG_STREAM("Scaled distance 1: " << d1 << " !< " << maxDist);
    DEBUG_STREAM("Scaled distance 2: " << d2 << " !< " << maxDist);

    if ((d1 > maxDist) && (d2 > maxDist))
    {
        DEBUG_STREAM("That's bad!");
        return false;
    }

    // everything's fine: refine the edge!
    segment->assignedEdge = this;

    // compute edge's new endpoints by sorting all known endpoints with respect to some imaginary distant point
    Vector2D distantPoint = startPoint + 100000.0 * getNormal().determineNormal();
    Vector2D p[4];
    p[0] = startPoint;
    p[1] = endPoint;
    p[2] = remoteStartPoint;
    p[3] = remoteEndPoint;

    for (int i = 0; i < 3; i++)
    {
        double dI = p[i].getDistanceTo(distantPoint);

        for (int j = i + 1; j < 4; j++)
        {
            double dJ = p[j].getDistanceTo(distantPoint);

            if (dI > dJ)
            {
                Vector2D tempPoint = p[i];
                p[i] = p[j];
                p[j] = tempPoint;
                dI = dJ;
            }
        }
    }

    startPoint = p[0];
    endPoint = p[3];
    length = startPoint.getDistanceTo(endPoint);

    double newSegmentQuality = (matchingSegment.linearity*matchingSegment.stablePointDistance + segment->linearity*segment->stablePointDistance)/(matchingSegment.stablePointDistance + segment->stablePointDistance);
    // update the matching segment's orientation
    matchingSegment.updateNormal(0.5 * (p[0] + p[1]), 0.5 * (p[2] + p[3]));
    matchingSegment.linearity = newSegmentQuality;

    if (segment->onBorder == TOUCHES_ONE_BORDER)
        borderLevel = segment->onBorder;

    hasRefinement = true;

    DEBUG_STREAM("Things are cool!");
    return true;
}

//=================================================================================================
//!
//! EdgeMatcherEdgePair
//!
//=================================================================================================


//=================================================================================================
EdgeMatcherEdgePair::EdgeMatcherEdgePair(EdgeMatcherEdge* _rootEdge, Mat& _mask)
    : threshold(0)
{
    rootEdge = _rootEdge;
    blobs = _mask.clone();

    //compute edgePair axis as the mean edge direction
    computeAxis(rootEdge, rootEdge->pairingEdge, axisBase, axis);
    axisNormal = axis.determineNormal();
    openingAngle = CV_PI - acos(rootEdge->getNormal() * rootEdge->pairingEdge->getNormal());

    // compute virtual tips
    computeTips(rootEdge, rootEdge->pairingEdge, axisBase, axis, virtualStartTipLongEdge, virtualEndTipLongEdge, virtualStartTipShortEdge, virtualEndTipShortEdge);
    virtualLengthLongEdge = virtualStartTipLongEdge.getDistanceTo(virtualEndTipLongEdge); // HIer weiter

    // locate contour tips
    Size imageSize = blobs.size();
    Vector2D p1 = virtualStartTipLongEdge;
    contourStartTip = p1;

    while ((p1.x > 0) && (p1.x < imageSize.width) && (p1.y > 0) && (p1.y < imageSize.height) && ((blobs.at<uchar>((int) p1.y *blobs.cols + (int) p1.x)) != 0))
    {
        contourStartTip = p1;
        p1 -= axis;
    }

    Vector2D p2 = virtualEndTipLongEdge;
    contourEndTip = p2;
    int stepsRemaining = floor(p2.getDistanceTo(axisBase)); //do not extend the tip past edge line intersection (because then either the edges are wrong or the tip is wrong, resulting in an inconsistent edgePair with incosistent widths)

    while ((p2.x > 0) && (p2.x < imageSize.width) && (p2.y > 0) && (p2.y < imageSize.height) && (blobs.at<uchar>((int) p2.y *blobs.cols + (int) p2.x) != 0) && stepsRemaining-- > 0)
    {
        contourEndTip = p2;
        p2 += axis;
    }

    // some more assignments...
    rootEdge->assignedTool = this;
    rootEdge->pairingEdge->assignedTool = this;
    contourLength = contourStartTip.getDistanceTo(contourEndTip);

    //define edgePair type
    if (rootEdge->pairingEdge->type==EDGEMATCHER_EDGETYPE_IMGBORDER)
        edgePairingType = EDGEMATCHER_PAIRED_1EDGE_WITHBORDER;
    else if (contourLength >= edgeMatcherParameters->EDGEPAIR_MIN_LENGTH)
        edgePairingType = EDGEMATCHER_PAIRED_2EDGES;
    else if (contourLength >= edgeMatcherParameters->EDGEPAIR_MIN_LENGTH/2 && (rootEdge->borderLevel==TOUCHES_ONE_BORDER || rootEdge->pairingEdge->borderLevel==TOUCHES_ONE_BORDER))
        edgePairingType = EDGEMATCHER_PAIRED_WITH_SHORT_BORDER_TOUCHING_EDGES;
    else {
        edgePairingType = EDGEMATCHER_PAIRED_INVALID;
        rootEdge->assignedTool = rootEdge->pairingEdge->assignedTool = NULL;
    }
}


//=================================================================================================
void EdgeMatcherEdgePair::computeAxis(EdgeMatcherEdge *pEdgeA, EdgeMatcherEdge *pEdgeB, Vector2D &rAxisBase, Vector2D &rAxis)
{
    Vector2D &edge1A = pEdgeA->matchingSegment.stableStartPoint;
    Vector2D &edge1B = pEdgeA->matchingSegment.stableEndPoint;
    Vector2D &edge2A = pEdgeB->matchingSegment.stableStartPoint;
    Vector2D &edge2B = pEdgeB->matchingSegment.stableEndPoint;

    Vector2D edgePairAxisA = (edge1B - edge1A).normalize();
    Vector2D edgePairAxisB = (edge2B - edge2A).normalize();

    if (edgePairAxisA*edgePairAxisB < 0)
        rAxis = (edgePairAxisA - edgePairAxisB).normalize();
    else
        rAxis = (edgePairAxisB - edgePairAxisA).normalize();

    //find the base point as the intersection of edge lines (medial axis)
    double d, foo;

    //    CvPoint pointA1((int)edge1A.x, (int)edge1A.y);
    //    CvPoint pointA2((int)edge1B.x, (int)edge1B.y);
    //    CvPoint pointB1((int)edge2A.x, (int)edge2A.y);
    //    CvPoint pointB2((int)edge2B.x, (int)edge2B.y);

    if (!llIntersection(edge1A, edge1B, edge2A, edge2B, d, foo))
        //    if (!llIntersection(pointA1, pointA2, pointB1, pointB2, d, foo))
        rAxisBase = edge1A + d * (edge1B - edge1A); //(the intersection of edge lines)
    else
        rAxisBase = edge1B + rAxis.determineNormal() * ((edge2B - edge1B)*rAxis.determineNormal() / 2.0); //(middle point between the lines if they're parallel)
}


//=================================================================================================
void EdgeMatcherEdgePair::computeTips(EdgeMatcherEdge *pEdgeA, EdgeMatcherEdge *pEdgeB, Vector2D &rAxisBase, Vector2D &rAxis, Vector2D &rStartTipLongEdge, Vector2D &rEndTipLongEdge, Vector2D &rStartTipShortEdge, Vector2D &rEndTipShortEdge)
{

    // compute virtual tips
    Vector2D* p[4] = {
        &pEdgeA->startPoint, &pEdgeA->endPoint,
        &pEdgeB->startPoint, &pEdgeB->endPoint };

    double minD = +1e9;
    double maxD = -1e9;
    int maxIdx = pEdgeB->type==EDGEMATCHER_EDGETYPE_IMGBORDER ? 2 : 4; //skip over border edges in this part

    for (int i = 0; i < maxIdx; i++)
    {
        double d = (*p[i] - rAxisBase) * rAxis;

        if (d > maxD)
        {
            maxD = d;
        }

        if (d < minD)
        {
            minD = d;
        }
    }

    rStartTipLongEdge = rAxisBase + minD * rAxis;
    rEndTipLongEdge = rAxisBase + maxD * rAxis;


    // compute virtual tips with respecto to small edge
    Vector2D* p_short[2];
    if (pEdgeA->length < pEdgeB->length)
    {
        p_short[0] = &pEdgeA->startPoint;
        p_short[1] = &pEdgeA->endPoint;
    }
    else
    {
        p_short[0] = &pEdgeB->startPoint;
        p_short[1] = &pEdgeB->endPoint;
    }

    minD = +1e9;
    maxD = -1e9;
    maxIdx = 2;//pEdgeB->type==EDGEMATCHER_EDGETYPE_IMGBORDER ? 2 : 4; //skip over border edges in this part

    for (int i = 0; i < maxIdx; i++)
    {
        double d = (*p_short[i] - rAxisBase) * rAxis;

        if (d > maxD)
        {
            maxD = d;
        }

        if (d < minD)
        {
            minD = d;
        }
    }

    rStartTipShortEdge = rAxisBase + minD * rAxis;
    rEndTipShortEdge = rAxisBase + maxD * rAxis;

}


//=================================================================================================
void EdgeMatcherEdgePair::computeShape(EdgeMatcherEdge *pEdgeA, EdgeMatcherEdge *pEdgeB, Vector2D &rStartTip, Vector2D &rEndTip, std::vector< Vector2D > &rvCoord)
{
    Vector2D n1 = pEdgeA->getNormal();
    Vector2D c1 = pEdgeA->getCenter();

    // order is important!
    rvCoord.push_back(rStartTip + (rStartTip - c1)*n1*n1);
    rvCoord.push_back(rEndTip + (rEndTip - c1)*n1*n1);

    Vector2D n2 = pEdgeB->getNormal();
    Vector2D c2 = pEdgeB->getCenter();

    // order is important!
    rvCoord.push_back(rEndTip + (rEndTip - c2)*n2*n2);
    rvCoord.push_back(rStartTip + (rStartTip - c2)*n2*n2);
}


//=================================================================================================
double EdgeMatcherEdgePair::getWidth(Vector2D tip, double tipDistance)
{
    //parallel case:
    if (openingAngle<1e-20)
        return abs( (rootEdge->matchingSegment.stableEndPoint - rootEdge->pairingEdge->matchingSegment.stableEndPoint) * axisNormal )/2;

    //converging case:
    double distToBase = axisBase.getDistanceTo(tip) - tipDistance;
    return sin(openingAngle/2)*distToBase;

    //note: this assumes, that even if the tips are extended (contourStart/End), the original pairing edges (at least
    //their normals) are still valid.
}


//=================================================================================================
bool EdgeMatcherEdgePair::pointIsBetweenTips(Vector2D coordinates)
{
    double dContourStart = contourStartTip.getDistanceTo(coordinates);
    double dContourEnd = contourEndTip.getDistanceTo(coordinates);

    return ((dContourStart <= contourLength) && (dContourEnd <= contourLength));
}


//=================================================================================================
void EdgeMatcherEdgePair::renderToolMask(Mat& maskImage, double widthCorrection/*=0*/)
{
    maskImage.setTo(0);

    Vector2D startPt = contourStartTip;
    Vector2D endPt = contourEndTip;
    double startW = max<double>(1, getWidth(startPt, 0) + widthCorrection);
    double endW = max<double>(1, getWidth(endPt, 0) + widthCorrection);

    // edgePair width in virtual tip
    //cvCircle(maskImage, startPt.toCvPoint(), (int)startW, white, -1);
    //cvCircle(maskImage, endPt.toCvPoint(), (int)endW, white, -1);

    // area between virtual tips
    vector<Point> points;
    points[0] = (startPt + rootEdge->getNormal()*startW).toCvPoint();
    points[1] = (startPt + rootEdge->pairingEdge->getNormal()*startW).toCvPoint();
    points[2] = (endPt + rootEdge->pairingEdge->getNormal()*endW).toCvPoint();
    points[3] = (endPt + rootEdge->getNormal()*endW).toCvPoint();
    fillConvexPoly(maskImage, points, Scalar(255));
}


//=================================================================================================
void EdgeMatcherEdgePair::renderToolEdges(Mat& maskImage, int thickness, int skipLine/*=0*/, float * normalAngle/*=0*/)
{
   maskImage.setTo(0);

    Vector2D startPt = contourStartTip;
    Vector2D endPt = contourEndTip;
    double startW = max<double>(1, getWidth(startPt, 0));
    double endW = max<double>(1, getWidth(endPt, 0));

    // lines between virtual tips outer edges
    for (int nline=1; nline<=2; nline++)
    {
        if (skipLine==nline) continue;
        Vector2D normal = (nline==1) ? rootEdge->getNormal() : rootEdge->pairingEdge->getNormal();

        Vector2D pt1 = startPt + normal*startW;
        Vector2D pt2 = endPt + normal*endW;
        line(maskImage, pt1.toCvPoint(), pt2.toCvPoint(), Scalar(255), thickness);

        if (normalAngle && skipLine!=0) {
            Vector2D diff(pt2.x-pt1.x, -pt2.y+pt1.y);
            if (diff*normal<0) diff *= -1;
            *normalAngle = atan2(diff.y, diff.x);
        }
    }
}


//=================================================================================================
double EdgeMatcherEdgePair::axisPtDist(const Vector2D & pt) const
{
    double c = - axisNormal.x * axisBase.x - axisNormal.y * axisBase.y;
    return fabs(axisNormal.x * pt.x + axisNormal.y * pt.y + c) / sqrt(axisNormal.x*axisNormal.x + axisNormal.y*axisNormal.y);
}


//=================================================================================================
bool EdgeMatcherEdgePair::isToolDuplicit(EdgeMatcherEdgePair * tmpEdgePair, EdgeMatcherEdgePair * edgePair, int i/*=0*/, int j/*=0*/)
{
    double angle = ANGLE_DIFFERENCE_RAD( atan2(tmpEdgePair->axis.y, tmpEdgePair->axis.x), atan2(edgePair->axis.y, edgePair->axis.x) ) * RAD_TO_DEG;
    if (angle>90) angle = 180 - angle;
    double distA = std::min<double>( tmpEdgePair->axisPtDist(edgePair->contourStartTip), tmpEdgePair->axisPtDist(edgePair->contourEndTip) );
    double distB = std::min<double>( edgePair->axisPtDist(tmpEdgePair->contourStartTip), edgePair->axisPtDist(tmpEdgePair->contourEndTip) );
    double dist = std::min<double>( distA, distB );
    ////cout << i << " " << j << " " << angle << " " << dist << endl;

    return (angle < 15 && dist < 30);
}

//=================================================================================================
//!
//! EdgeMatcherIntersection
//!
//=================================================================================================

//=================================================================================================
EdgeMatcherIntersection::EdgeMatcherIntersection(EdgeMatcherEdgePair* _a, EdgeMatcherEdgePair* _b, Vector2D _coordinates)
{
    a = _a;
    b = _b;
    coordinates = _coordinates;
}

//=================================================================================================
//!
//! EdgeMatcherContourPoint
//!
//=================================================================================================

//=================================================================================================
EdgeMatcherContourPoint::EdgeMatcherContourPoint()
{
}

//=================================================================================================
void EdgeMatcherContourPoint::setBase(int _index, Point &_coord)
{
    index = _index;

    coord.x = _coord.x;
    coord.y = _coord.y;
}

//=================================================================================================
void EdgeMatcherContourPoint::setAngle(Vector2D &_vectorDiffAhead, Vector2D &_vectorDiffBehind)
{
    angleAhead = RAD_TO_DEG*atan2(_vectorDiffAhead.y, _vectorDiffAhead.x);
    angleBehind = RAD_TO_DEG*atan2(_vectorDiffBehind.y, _vectorDiffBehind.x);
    angleDifference = ANGLE_DIFFERENCE_DEG(angleAhead, angleBehind);
    declaredAsLine = false;

    Vector2D summOfDiff = _vectorDiffAhead + _vectorDiffBehind;

    angle = RAD_TO_DEG*atan2(summOfDiff.y, summOfDiff.x);
    normal = summOfDiff.normalize().determineNormal();
}

//=================================================================================================
double EdgeMatcherContourPoint::getAngle(bool bForward)
{
    if (bForward)
        return angleBehind;
    else
        return angleAhead;
}

//=================================================================================================
bool EdgeMatcherContourPoint::compareUsingAngle(EdgeMatcherContourPoint *a, EdgeMatcherContourPoint *b)
{
    return (a->angleDifference < b->angleDifference);
}


//=================================================================================================
//!
//! EdgeMatcherDynamicThreshold
//!
//=================================================================================================

//=================================================================================================
EdgeMatcherDynamicThreshold::EdgeMatcherDynamicThreshold()
{
}

//=================================================================================================
EdgeMatcherDynamicThreshold::EdgeMatcherDynamicThreshold(double _parLeft, double _valLeft, double _parRight, double _valRight)
    : parLeft(_parLeft)
    , valLeft(_valLeft)
    , parRight(_parRight)
    , valRight(_valRight)
    , scale((valRight - valLeft)/(parRight - parLeft))
{
}

//=================================================================================================
double EdgeMatcherDynamicThreshold::operator () (const double _par)
{
    if (_par < parLeft)
        return valLeft;
    else if (_par > parRight)
        return valRight;

    return valLeft + (_par - parLeft)*scale;
}
