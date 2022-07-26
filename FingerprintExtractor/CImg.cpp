#include "CImg.h"

// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>::CImg()
{
  data = NULL;
}

// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>::CImg(int _dimX, int _dimY, int _planes)
{
  dimX = _dimX;
  dimY = _dimY;
  planes = _planes;
  data = NULL;
  data = new T [dimX * dimY * planes];
}

// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>::CImg(int _dimX, int _dimY, int _planes, T* _data)
{
  dimX = _dimX;
  dimY = _dimY;
  planes = _planes;
  data = NULL;
  data = new T [dimX * dimY * planes];
  memcpy(data, _data, dimX * dimY * planes * sizeof(T));
}


// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>::CImg(int _dimX, int _dimY, int _planes, T val)
{
  dimX = _dimX;
  dimY = _dimY;
  planes = _planes;
  data = NULL;
  data = new T [dimX * dimY * planes];
  memset(data, val, dimX * dimY * planes * sizeof(T));
}

// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>::CImg(cv::Mat ocvMat)
{
  dimX = ocvMat.cols;
  dimY = ocvMat.rows;
  planes = ocvMat.channels();

  data = new T [dimX * dimY * planes];
  memset(data, (T)0, dimX * dimY * planes * sizeof(T));

  if(ocvMat.channels() == 1)
  {
    for(int i = 0; i < ocvMat.rows; i++)
    {
      for(int j = 0; j < ocvMat.cols; j++)
      {
        data[i * ocvMat.cols + j] = vPixl(&ocvMat, i, j);
      }
    }
  }
  else
  {
    for(int i = 0; i < ocvMat.rows; i++)
    {
      for(int j = 0; j < ocvMat.cols; j++)
      {
        data[i * ocvMat.cols + j]                                 = rPixl(&ocvMat, i, j);
        data[ocvMat.cols * ocvMat.rows + i * ocvMat.cols + j]     = gPixl(&ocvMat, i, j);
        data[2 * ocvMat.cols * ocvMat.rows + i * ocvMat.cols + j] = bPixl(&ocvMat, i, j);
      }
    }
  }
}


// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>::~CImg()
{
  if(data)
  {
    delete [] data;
  }
  //    if (dimX>0 || dimY>0 || planes>0)

  dimX = 0;
  dimY = 0;
  planes = 0;
}

// --------------------------------------------------------------------------------------------
template <class T>
CImg<T>& CImg<T>::operator=(const CImg<T>& other)
{
  if(data)
  {
    delete [] data;
  }

  dimX = other.dimX;
  dimY = other.dimY;
  planes = other.planes;
//  data = other.data;
  data = new T [dimX * dimY * planes];
  memcpy(data, other.data, dimX * dimY * planes * sizeof(T));

  return *this;
}


// --------------------------------------------------------------------------------------------
template <class T>
T CImg<T>::vPixl(cv::Mat* ocvMat, int i, int j)
{
  return ocvMat->at<T>(i, j);
}


// --------------------------------------------------------------------------------------------
template <class T>
T CImg<T>::bPixl(cv::Mat* ocvMat, int i, int j)
{
  return getPixl(ocvMat, i, j, 0);
}


// --------------------------------------------------------------------------------------------
template <class T>
T CImg<T>::gPixl(cv::Mat* ocvMat, int i, int j)
{
  return getPixl(ocvMat, i, j, 1);
}


// --------------------------------------------------------------------------------------------
template <class T>
T CImg<T>::rPixl(cv::Mat* ocvMat, int i, int j)
{
  return getPixl(ocvMat, i, j, 2);
}


// --------------------------------------------------------------------------------------------
template <class T>
T CImg<T>::getPixl(cv::Mat* ocvMat, int i, int j, int dim)
{
  T val;
  switch ( ocvMat->depth() )
  {
  case CV_8U:
    val = ocvMat->at<cv::Vec3b>(i, j)[dim];
    break;
  case CV_8S:
    val = ocvMat->at<cv::Vec3b>(i, j)[dim];
    break;
  case CV_16U:
    val = ocvMat->at<cv::Vec3s>(i, j)[dim];
    break;
  case CV_16S:
    val = ocvMat->at<cv::Vec3s>(i, j)[dim];
    break;
  case CV_32S:
    val = ocvMat->at<cv::Vec3i>(i, j)[dim];
    break;
  case CV_32F:
    val = ocvMat->at<cv::Vec3f>(i, j)[dim];
    break;
  case CV_64F:
    val = ocvMat->at<cv::Vec3d>(i, j)[dim];
    break;
  }
  return val;
}

// --------------------------------------------------------------------------------------------
template <class T>
cv::Mat CImg<T>::cvt2OCVImg(void)
{
  T* image = data;
  int width = dimX;
  int height = dimY;
  cv::Mat imgOut;

  if(planes == 3)
  {
    //  ... for a 3-channel image
    cv::Mat img;
    if (typeid(T) == typeid(unsigned char))
      img.create(height, width, CV_8UC3);
    else if (typeid(T) == typeid(char))
      img.create(height, width, CV_8SC3);
    else if (typeid(T) == typeid(unsigned short))
      img.create(height, width, CV_16UC3);
    else if (typeid(T) == typeid(short))
      img.create(height, width, CV_16SC3);
    else if (typeid(T) == typeid(int))
      img.create(height, width, CV_32SC3);
    else if (typeid(T) == typeid(float))
      img.create(height, width, CV_32FC3);
    else if (typeid(T) == typeid(double))
      img.create(height, width, CV_64FC3);

    int i, j;
    T* pIn  = image;
    T* pIn2 = image + width * height;
    T* pIn3 = image + 2 * width * height;
    for(i = 0; i < height; i++)
    {
      T* pOut = img.ptr<T>(i);
      for(j = 0; j < width; j++)
      {
        *pOut++ = (T)(*pIn3++);  // B
        *pOut++ = (T)(*pIn2++);  // G
        *pOut++ = (T)(*pIn++);   // R
      }
    }
    imgOut = img.clone();
  }
  else if(planes == 1)
  {
    //  ... for a 1-channel image
    cv::Mat img;
    if (typeid(T) == typeid(unsigned char))
      img.create(height, width, CV_8U);
    else if (typeid(T) == typeid(char))
      img.create(height, width, CV_8S);
    else if (typeid(T) == typeid(unsigned short))
      img.create(height, width, CV_16U);
    else if (typeid(T) == typeid(short))
      img.create(height, width, CV_16S);
    else if (typeid(T) == typeid(int))
      img.create(height, width, CV_32S);
    else if (typeid(T) == typeid(float))
      img.create(height, width, CV_32F);
    else if (typeid(T) == typeid(double))
      img.create(height, width, CV_64F);

    int i, j;
    T* pIn = image;
    for(i = 0; i < height; i++)
    {
      T* pOut = img.ptr<T>(i);
      for(j = 0; j < width; j++)
      {
        *pOut++  = (T)(*pIn++);
      }
    }
    imgOut = img.clone();
  }

  return imgOut;
}


// --------------------------------------------------------------------------------------------
// provide the following specific CImg types
template class CImg<unsigned char>;
template class CImg<char>;
template class CImg<unsigned int>;
template class CImg<int>;
template class CImg<float>;
template class CImg<double>;

