#include "sobelEdgeStrength.h"

void sobelEdgeStrength(Mat& src, Mat& grad, const int kernelsize, const double scale)
{
    Mat src_gray;
    if (src.channels() == 3)
    {
        cvtColor(src, src_gray, CV_BGR2GRAY);
    }
    else if (src.channels() == 1)
    {
        src_gray = src;
    }
    else
    {
        return;
    }

    /// Sobel filtering
    ///
    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y; //, grad;
//    int scale = 2;
    int delta = 0;
    int ddepth = CV_16S;
//    int kernelsize = 3;
    cv::Sobel( src_gray, grad_x, ddepth, 1, 0, kernelsize, scale, delta, cv::BORDER_DEFAULT );
    cv::Sobel( src_gray, grad_y, ddepth, 0, 1, kernelsize, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );
    cv::convertScaleAbs( grad_y, abs_grad_y );
    // Edge strength (approximate)
    cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
}
