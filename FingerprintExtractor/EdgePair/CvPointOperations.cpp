//*****************************************************************************
// Methoden von CvPointOPs als Erweiterung zu CvPoint
//*****************************************************************************

#include "CvPointOperations.h"
#include <opencv2/core/types_c.h>
#include <opencv2/opencv.hpp>


////Konstruktoren
CvPointOPs::CvPointOPs()
{
    x = 0;
    y = 0;
}

CvPointOPs::CvPointOPs(int a)
{
    x = a;
    y = a;
}

CvPointOPs::CvPointOPs(int a, int b)
{
    x = a;
    y = b;
}


//geht nicht!
//CvPointOPs::CvPointOPs(CvPoint p) : x(p.x), y(p.y)
//{
//}

CvPointOPs::CvPointOPs(Point p)
{
    x = p.x;
    y = p.y;
}

CvPointOPs::CvPointOPs(const CvPointOPs& rhs)
{
    x = rhs.x;
    y = rhs.y;
}


CvPointOPs::~CvPointOPs()
{
}

/********************
/bin&auml;re Operatoren
****************************/
CvPointOPs& CvPointOPs::operator= (const Point& rhs)
{
    if (this == &rhs)
    {
        return *this;
    }

    x = rhs.x;
    y = rhs.y;
    return *this;
}

CvPointOPs CvPointOPs::operator+ (const Point& rhs)
{
    return cv::Point(x + rhs.x, y + rhs.y);
}

CvPointOPs CvPointOPs::operator- (const Point& rhs)
{
    return cv::Point(x - rhs.x, y - rhs.y);
}

CvPointOPs& CvPointOPs::operator+=(const CvPointOPs& rhs)
{
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
}

CvPointOPs& CvPointOPs::operator-=(const CvPointOPs& rhs)
{
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
}

CvPointOPs CvPointOPs::operator* (const int& n)
{
    return cv::Point(x * n, y * n);
}

CvPointOPs CvPointOPs::operator/ (const int& n)
{
    return cv::Point(x / n, y / n);
}

/********************
/Vergleichsoperatoren
****************************/

bool CvPointOPs::operator== (const Point& rhs) const
{
    if (x == rhs.x && y == rhs.y)
    {
        return true;
    }

    return false;
}

bool CvPointOPs::operator== (const int& n) const
{
    if (sqrt((double)(x * x + y * y)) == n)
    {
        return true;
    }

    return false;
}

bool CvPointOPs::operator== (const double& n) const
{
    if (sqrt((double)(x * x + y * y)) == n)
    {
        return true;
    }

    return false;
}

//bool CvPointOPs::operator== (const float& n) const



bool CvPointOPs::operator!= (const Point& rhs) const
{
    if (x != rhs.x || y != rhs.y)
    {
        return true;
    }

    return false;
}

bool CvPointOPs::operator!= (const int& n) const
{
    if (sqrt((double)(x * x + y * y)) != n)
    {
        return true;
    }

    return false;
}

bool CvPointOPs::operator!= (const double& n) const
{
    if (sqrt((double)(x * x + y * y)) != n)
    {
        return true;
    }

    return false;
}

bool CvPointOPs::operator > (const CvPointOPs& rhs) const
{
   return ( sqrt((double)(x*x + y*y)) >  sqrt((double)(rhs.x*rhs.x + rhs.y*rhs.y))   );
//   return abs() > rhs.abs();
}

bool CvPointOPs::operator < (const CvPointOPs& rhs) const
{
    return ( sqrt((double)(x*x + y*y)) <  sqrt((double)(rhs.x*rhs.x + rhs.y*rhs.y))   );
//    return abs() < rhs.abs();
}

//-----------------------------------------------------------------------------
//! bildet den Absolutbetrag, wenn der Punkt als Spitze eines Vektors
//! angesehen wird, der im Ursprung beginnt.
//-----------------------------------------------------------------------------
double CvPointOPs::abs()
{
    return sqrt((double)(x * x + y * y));
}

double CvPointOPs::dist(CvPointOPs dest)
{
    return sqrt((double)(dest.x - x)* (dest.x-x) + (dest.y -y)*(dest.y - y));
}

ostream& operator<<(ostream& out, const CvPointOPs& p)
{
    out << "(" << p.x << "/" << p.y << ")";
    return out;
}

// so geht's nicht!
//ostream& CvPointOPs::operator<<(ostream& out)
//{
//        out << "(" << x << "/" << y << ")" << endl;
//        return out;
//}

bool CvPointOPs::isPointInBox(Point p, CvBox2D box)
{
//    CvPoint2D32f points[4];
    Point points[4];
    double triangleArea[4], totalArea = 0.0;
    //cvBoxPoints(box, &points[0]);
    Point  EdgePoints[] = { {(int)points[0].x, (int)points[0].y},
        {(int)points[1].x, (int)points[1].y},
        {(int)points[2].x, (int)points[2].y},
        {(int)points[3].x, (int)points[3].y}
    };
    Point* EdgeArr[1] = { EdgePoints };
    int countPosOrientation = 0;

    for (int i = 0; i < 4; i++)
    {
        //        std:: cout << "p["<<i<<"]=("<<points[i].x<<", "<<points[i].y<<"), ";
        triangleArea[i] = 0.5 * (
                              points[i].x * points[(i + 1) % 4].y
                              + points[i].y * p.x
                              + points[(i + 1) % 4].x * p.y
                              - p.x * points[(i + 1) % 4].y
                              - p.y * points[i].x
                              - points[(i + 1) % 4].x * points[i].y
                          );

        //        std::cout << "A[" << i << "] = " << triangleArea[i] << std::endl;
        totalArea += triangleArea[i] ;

        if (triangleArea[i] > 0.0)
        {
            countPosOrientation++;
        }
    }

    //    std:: cout << "p= ("<<p.x<<","<<p.y<<")"<<std::endl;
    //    std::cout << "A_ges = " << totalArea << std::endl;
    return (countPosOrientation == 0 || countPosOrientation == 4);
}
