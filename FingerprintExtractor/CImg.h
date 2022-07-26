#ifndef _CIMG_H_
#define _CIMG_H_

// OpenCV
#include <opencv2/opencv.hpp>

template <class T>
class CImg
{
public:
  CImg();
  CImg(cv::Mat ocvMat);
  CImg(int _dimX, int _dimY, int _planes);
  CImg(int _dimX, int _dimY, int _planes, T *_data);
  CImg(int _dimX, int _dimY, int _planes, T val);
  ~CImg();
  CImg<T>& operator=(const CImg<T>& other);

  int dimX;
  int dimY;
  int planes;
  T* data;

  cv::Mat cvt2OCVImg(void);

private:
  T vPixl(cv::Mat* ocvMat, int i, int j);
  T bPixl(cv::Mat* ocvMat, int i, int j);
  T gPixl(cv::Mat* ocvMat, int i, int j);
  T rPixl(cv::Mat* ocvMat, int i, int j);
  T getPixl(cv::Mat* ocvMat, int i, int j, int dim);

};



#endif   // _CIMG_H_
