// =============================================================================
// INCLUDES
// =============================================================================

#include <iostream>
#include <math.h>
#include <stdlib.h>
//#include <QStringList>
#include "CFill.h"
#include "CBlobLabel.h"
#include "CPalmSegmentation.h"
#include "CImg.h"

#define M_PI 3.141528

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

CPalmSegmentation::CPalmSegmentation()
{
  ;// (noch) nichts zu tun
}

CPalmSegmentation::~CPalmSegmentation()
{
	;// (noch) nichts zu tun
}




void CPalmSegmentation::getSkinColorMask(cv::Mat inFrameScaled, cv::Mat &skinColorMask_ocv, int &BlobSize)
{
    BlobSize = 0;
    skinColorMask_ocv.create(inFrameScaled.rows, inFrameScaled.cols, CV_8UC1);

    CImg<unsigned char> normImage(inFrameScaled);
    unsigned char* mask_image = new unsigned char[normImage.dimX*normImage.dimY];
    memset(mask_image, 0, normImage.dimX*normImage.dimY);
    //QRect detectorROI = QRect(0, 0, normImage.dimX, normImage.dimY);
    getHandPalmBlob(normImage.data, normImage.dimX, normImage.dimY, normImage.planes, mask_image);//, detectorROI);

    for(int i = 0; i < normImage.dimY; i++)
    {
        unsigned char* pOut = skinColorMask_ocv.ptr(i);
        for(int j = 0; j < normImage.dimX; j++)
        {
            if (mask_image[j + normImage.dimX*i] >= 1)
            {
                *pOut++ = 255;
                BlobSize++;
            }
            else
            {
                *pOut++ = 0;
            }
        }
    }

//    unsigned char* data = new unsigned char[inFrameScaled.cols*inFrameScaled.rows*3];
//    memset(data, 0, inFrameScaled.cols*inFrameScaled.rows*3);
//    unsigned char* mask_image = new unsigned char[inFrameScaled.cols*inFrameScaled.rows];
//    memset(mask_image, 0, inFrameScaled.cols*inFrameScaled.rows);
//    //QRect detectorROI = QRect(0, 0, normImage.dimX, normImage.dimY);
//    getHandPalmBlob(data, inFrameScaled.cols, inFrameScaled.rows, 3, mask_image);//, detectorROI);


//    for(int i = 0; i < inFrameScaled.rows; i++)
//    {
//        unsigned char* pOut = skinColorMask_ocv.ptr(i);
//        for(int j = 0; j < inFrameScaled.cols; j++)
//        {
//            if (mask_image[j + inFrameScaled.cols*i] >= 1)
//            {
//                *pOut++ = 255;
//                BlobSize++;
//            }
//            else
//            {
//                *pOut++ = 0;
//            }
//        }
//    }

//    delete [] data;

    delete [] mask_image;

}


void CPalmSegmentation::getHandPalmBlob(unsigned char* image, int width, int height, int planes, unsigned char* mask) //, QRect HandBB)
{
    memset(mask, 0, width*height);
    unsigned char* skinColorMask=new unsigned char[width*height];

  // 1) Find Pixel With Skin Color
    unsigned char* temp = new unsigned char[width*height];

    skinColorSegmentation(image, width, height, planes, skinColorMask);

  // 2) Remove Areas outside ROI
#if 0
    for(int yi=0; yi<height; yi++)
    for(int xi=0; xi<width; xi++)
    {
      if(!HandBB.contains(xi, yi))
        skinColorMask[xi+yi*width]=0;
    }
#endif

  // 2) Do Blob Clustering and Remove Small Clutter
  //    It is assumed, that Hand is a significant part of the image (e.g. >5-10% of image size)

    int AnzBlobs=0;
    CBlobLabel myBlobLabeling;
    int* BlobImage = new int[width*height];
    memset(BlobImage, 0, width*height);
    myBlobLabeling.BlobAnalysisIntOpt(skinColorMask, width, height, BlobImage, 0, AnzBlobs);

    int* BlobSize=new int[AnzBlobs];
    memset(BlobSize, 0, AnzBlobs*sizeof(int));

    for(int yi=0; yi<height; yi++)
        for(int xi=0; xi<width; xi++)
        {
            if(BlobImage[xi+yi*width]>0)
                BlobSize[BlobImage[xi+yi*width]-1]++;
        }

#if 0
    for(int yi=0; yi<height; yi++)
        for(int xi=0; xi<width; xi++)
        {
            // if(BlobImage[xi+yi*width]>0 && BlobSize[BlobImage[xi+yi*width]-1]<HandBB.width()*HandBB.height()/5/*5%*/)
            //if(BlobImage[xi+yi*width]>0 && BlobSize[BlobImage[xi+yi*width]-1]<HandBB.width()*HandBB.height()/50/*5%*/)
            if(0)
            {
                BlobImage[xi+yi*width]=0;
                skinColorMask[xi+yi*width]=0;
            }
        }
#endif

    // Redo BlobLabeling for Data Consistancy
    memset(BlobImage, 0, width*height);
    myBlobLabeling.BlobAnalysisIntOpt(skinColorMask, width, height, BlobImage, 0, AnzBlobs);


  // 3) Find Major Blob inside Detector ROI
    /*Erosion myEro;
    for(int it=0; it<2; it++)
    {
      myEro.work(skinColorMask, temp, width, height, 3);
      memcpy(skinColorMask, temp, width*height);
    }*/

    /*Dilatation myDil;
    for(int it=0; it<2; it++)
    {
      myDil.work(skinColorMask, temp, width, height, 3);
      memcpy(skinColorMask, temp, width*height);
    }*/


    int* BlobHisto=new int[AnzBlobs];
    memset(BlobHisto, 0, AnzBlobs*sizeof(int));
    for(int yi=0; yi<height; yi++)
    for(int xi=0; xi<width; xi++)
    {
      //if(BlobImage[xi+yi*width]>0 && xi>HandBB.x() && yi>HandBB.y() && xi<HandBB.x()+HandBB.width() && yi<HandBB.y()+HandBB.height())
      if(BlobImage[xi+yi*width]>0)
        BlobHisto[BlobImage[xi+yi*width]-1]++;
    }

    int BlobIdx=0;
    int Max=0;
    for(int i=0; i<AnzBlobs; i++)
    {
      if(BlobHisto[i]>Max)
      {
        Max=BlobHisto[i];
        BlobIdx=i+1;
      }
    }

    // 4) Prepare mask Image as Output
    if(BlobIdx>0)
    {
      for(int i=0; i<width*height; i++)
      {
        //if(BlobIdx==BlobImage[i])
        if(BlobImage[i]>0) // take into account all segmented blobs!!!
          mask[i]=255;
        else
          mask[i]=0;
      }
    }

    // 5) Remove "holes"
    Cfill FillH;
    FillH.fill(mask, width, height, 5, 5, 128, 0);

    for(int i=0; i<width*height; i++)
    {
      if(mask[i]==128)
        mask[i]=0;
      else
        mask[i]=255;
    }

    /*Dilatation dil;
    unsigned char* dil_image=new unsigned char[width*height];
    dil.work(mask, dil_image, width, height, 5);
    Erosion ero;
    ero.work(dil_image, mask, width, height, 3);

    FillH.fill(mask, width, height, 5, 5, 128, 0);
    for(int i=0; i<width*height; i++)
    {
      if(mask[i]==128)
        mask[i]=0;
      else
        mask[i]=255;
    }

    delete [] dil_image;*/

    // 5) Memory CleanUp
    delete [] BlobImage;
    delete [] skinColorMask;
    delete [] temp;
    delete [] BlobHisto;
    delete [] BlobSize;

}


void CPalmSegmentation::skinColorSegmentation(unsigned char* image, int width, int height, int planes, unsigned char* mask)
{
  // http://www.paulmckevitt.com/pubs/icip09.pdf

  double GreyConvertVec[3];
  GreyConvertVec[0]=0.298936021293775390;
  GreyConvertVec[1]=0.587043074451121360;
  GreyConvertVec[2]=0.140209042551032500;

  memset(mask, 0, width*height);

  if(planes!=3)
    return;

  double* dataD=new double[width*height*planes];
  double* dataD_I=new double[width*height];
  double* dataD_II=new double[width*height];
  double* eMask=new double[width*height];

  for (int i = 0; i < width * height * planes; i++)
  {
    dataD[i]=image[i];
    dataD[i]/=255.0;
  }

  for (int i = 0; i < width * height; i++)
  {
    dataD_I[i]=GreyConvertVec[0]*dataD[i]+GreyConvertVec[1]*dataD[i+width*height]+GreyConvertVec[2]*dataD[i+width*height*2];
    dataD_II[i]=dataD[i+width*height];
    if(dataD[i+width*height*2]>dataD[i+width*height])
      dataD_II[i]=dataD[i+width*height];

    eMask[i]=dataD_I[i]-dataD_II[i];
  }

  for (int i = 0; i < width * height; i++)
  {
    if(eMask[i]>=0.02511 && eMask[i]<=0.1177)
      mask[i]=255;
    else
      mask[i]=0;
  }


  delete [] dataD;
  delete [] dataD_I;
  delete [] dataD_II;
  delete [] eMask;
}
