#include <opencv2/opencv.hpp>
using namespace cv;

void sobelEdgeStrength(Mat& src, Mat& grad, const int kernelsize = 3, const double scale = 1);
