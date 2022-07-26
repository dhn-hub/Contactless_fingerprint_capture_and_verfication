#ifndef CPALMSEGMENTATION_H_
#define CPALMSEGMENTATION_H_

#include <opencv2/opencv.hpp>

#include <iostream>

class CPalmSegmentation
{
public:

  CPalmSegmentation();
  ~CPalmSegmentation();

  void getSkinColorMask(cv::Mat inFrameScaled, cv::Mat &skinColorMask_ocv, int &BlobSize);
  
  void getHandPalmBlob(unsigned char* image, int width, int height, int planes, unsigned char* mask);//, QRect HandBB);
  void skinColorSegmentation(unsigned char* image, int width, int height, int planes, unsigned char* mask);
private:


};

#endif /*CPALMSEGMENTATION_H_*/

