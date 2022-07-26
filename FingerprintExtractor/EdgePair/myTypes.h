#ifndef __MYTYPES_H
#define __MYTYPES_H
#include "opencv2/opencv.hpp"

using namespace cv;

#if defined(uchar)
#else
typedef unsigned char uchar;
#endif
#if defined(ushort)
#else
typedef unsigned short ushort;
#endif
#if defined(uint)
#else
typedef unsigned int uint;
#endif
#if defined(ulong)
#else
typedef unsigned long ulong;
#endif


#include <math.h>
#ifndef CV_PI
#define CV_PI   3.1415926535897932384626433832795
#endif

/*=============================== Makros ===================================*/

//! @name RGB Farbwerte (ui8)
//@{
#define red         CV_RGB(255,0,0)     //!< red
#define red50       CV_RGB(50,0,0)      //!< red50
#define green       CV_RGB(0,255,0)     //!< gruen
#define blue        CV_RGB(0,0,255)     //!< blau
#define cyan        CV_RGB(255,255,0)   //!< cyan
#define magenta     CV_RGB(255,0,255)   //!< magenta
//@}

//! @name Grauwerte (ui8)
//@{
#define black       cvScalarAll(0)      //!< schwarz
#define grey10      cvScalarAll(25)     //!< Grauwert (10% von ui8)
#define grey20      cvScalarAll(51)     //!< Grauwert (20% von ui8)
#define grey30      cvScalarAll(77)     //!< Grauwert (30% von ui8)
#define grey40      cvScalarAll(102)    //!< Grauwert (40% von ui8)
#define grey50      cvScalarAll(127)    //!< Grauwert (50% von ui8)
#define grey60      cvScalarAll(153)    //!< Grauwert (60% von ui8)
#define grey70      cvScalarAll(179)    //!< Grauwert (70% von ui8)
#define grey80      cvScalarAll(204)    //!< Grauwert (80% von ui8)
#define grey90      cvScalarAll(230)    //!< Grauwert (90% von ui8)
#define white       cvScalarAll(255)    //!< white
//@}

//-----------------------------------------------------------------------------
//! Sign function: http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
//-----------------------------------------------------------------------------
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//-----------------------------------------------------------------------------
//! Returns the unsigned smallest angular difference between any a and b. It's always within [0,360)
//-----------------------------------------------------------------------------
inline double ANGLE_DIFFERENCE_DEG(double a, double b)
{
    double d = fmod(fabs(a - b), 360.0);
    return d > 180.0 ? 360.0 - d : d;
}
//-----------------------------------------------------------------------------
//! Returns the unsigned smallest angular difference between any a and b. It's always within [0,2PI)
//-----------------------------------------------------------------------------
inline double ANGLE_DIFFERENCE_RAD(double a, double b)
{
    double d = fmod(fabs(a - b), 2*CV_PI);
    return d > CV_PI ? 2*CV_PI - d : d;
}


#if 1
class Vector2D
{
public:

    // Members
    double x;   //!< X coordinate
    double y;   //!< Y coordinate

    //-----------------------------------------------------------------------------
    //! @brief Simple default constructor
    //!
    //-----------------------------------------------------------------------------
    inline Vector2D()
    {
        x = 0.0;
        y = 0.0;
    }

    //-----------------------------------------------------------------------------
    //! @brief Constructor taking x and y coordinates
    //!
    //! @param[in]  _x  The vector's x coordinate
    //! @param[in]  _y  The vector's y coordinate
    //-----------------------------------------------------------------------------
    inline Vector2D(const double _x, const double _y)
    {
        x = _x;
        y = _y;
    }

    //-----------------------------------------------------------------------------
    //! @brief Copy constructor
    //!
    //! @param[in]  a   The vector to copy
    //-----------------------------------------------------------------------------
    inline Vector2D(const Vector2D &a)
    {
        x = a.x;
        y = a.y;
    }

    //-----------------------------------------------------------------------------
    //! @brief Constructor taking an OpenCV integer point
    //!
    //! @param[in]  p   The OpenCV point based on integer coordinates
    //-----------------------------------------------------------------------------
    inline Vector2D(Point p)
    {
        x = p.x;
        y = p.y;
    }

//    //-----------------------------------------------------------------------------
//    //! @brief Constructor taking an OpenCV floating point point
//    //!
//    //! @param[in]  p   The OpenCV point based on floating point coordinates
//    //-----------------------------------------------------------------------------
//    inline Vector2D(Point p)
//    {
//        x = p.x;
//        y = p.y;
//    }

    //-----------------------------------------------------------------------------
    //! @brief Assignment operator taking an OpenCV integer point
    //!
    //! @param[in]  b   The OpenCV integer point to assign
    //-----------------------------------------------------------------------------
    inline Vector2D operator = (const Point &b)
    {
        return Vector2D(b.x, b.y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Assignment operator taking an OpenCV floating point point
    //!
    //! @param[in]  b   Then OpenCV floating point point to assign
    //-----------------------------------------------------------------------------
    inline Vector2D operator = (const CvPoint2D32f &b)
    {
        return Vector2D(b.x, b.y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for adding two vectors
    //!
    //! @param[in]  b   The vector to add
    //-----------------------------------------------------------------------------
    inline Vector2D operator + (const Vector2D &b)
    {
        return Vector2D(x + b.x, y + b.y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for substracting two vectors
    //!
    //! @param[in]  b   The vector to Substract
    //-----------------------------------------------------------------------------
    inline Vector2D operator - (const Vector2D &b)
    {
        return Vector2D(x - b.x, y - b.y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for scalar multiplication of a vector and a scalar
    //!
    //! @param[in]  a   The vector to multiply
    //! @param[in]  b   The scalar to multiply with
    //-----------------------------------------------------------------------------
    inline friend Vector2D operator * (const Vector2D &a, const double &b)
    {
        return Vector2D(a.x*b, a.y*b);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for scalar multiplication of a scalar and a vector
    //!
    //! @param[in]  a   The scalar to multiply with
    //! @param[in]  b   The vector to multiply
    //-----------------------------------------------------------------------------
    inline friend Vector2D operator * (const double &a, const Vector2D &b)
    {
        return Vector2D(a*b.x, a*b.y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for vector product of two vectors
    //!
    //! @param[in]  b   The vector for which the product should be computed
    //-----------------------------------------------------------------------------
    inline double operator * (const Vector2D &b)
    {
        return x*b.x + y*b.y;
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for scalar division of a vector and a scalar
    //!
    //! @param[in]  a   The vector to divide
    //! @param[in]  b   The scalar to divide by
    //-----------------------------------------------------------------------------
    inline friend Vector2D operator / (const Vector2D &a, const double &b)
    {
        return Vector2D(a.x/b, a.y/b);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for scalar division of a scalar and a vector
    //!
    //! @param[in]  a   The scalar to divide by
    //! @param[in]  b   The vector to divide
    //-----------------------------------------------------------------------------
    inline friend Vector2D operator / (const double &a, const Vector2D &b)
    {
        return Vector2D(b.x/a, b.y/a);
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for adding two vectors
    //!
    //! @param[in]  b   The vector to add
    //-----------------------------------------------------------------------------
    inline Vector2D & operator += (const Vector2D &b)
    {
        x += b.x;
        y += b.y;

        return *this;
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for substracting two vectors
    //!
    //! @param[in]  b   The vector to Substract
    //-----------------------------------------------------------------------------
    inline Vector2D & operator -= (const Vector2D &b)
    {
        x -= b.x;
        y -= b.y;

        return *this;
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for scalar multiplication with a scalar
    //!
    //! @param[in]  b   The scalar to multiply with
    //-----------------------------------------------------------------------------
    inline Vector2D & operator *= (const double &b)
    {
        x *= b;
        y *= b;

        return *this;
    }

    //-----------------------------------------------------------------------------
    //! @brief Operator for scalar division of a vector and a scalar
    //!
    //! @param[in]  b   The scalar to divide by
    //-----------------------------------------------------------------------------
    inline Vector2D & operator /= (const double &b)
    {
        x /= b;
        y /= b;

        return *this;
    }

    //-----------------------------------------------------------------------------
    //! @brief Compute the length of a vector
    //!
    //! @return The vector's length
    //-----------------------------------------------------------------------------
    inline double getLength()
    {
        return sqrt(x*x + y*y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Returns the vector scaled to a length of 1.0
    //!
    //! @return The normalized vector
    //-----------------------------------------------------------------------------
    inline Vector2D normalize()
    {
        double l = getLength();

        return Vector2D(x/l, y/l);
    }

    //-----------------------------------------------------------------------------
    //! @brief Determines an orthogonal vector to the current vector with a length of 1.0
    //!
    //! @return The orthogonal and normalized vector
    //-----------------------------------------------------------------------------
    inline Vector2D determineNormal()
    {
        double l = getLength();

        return Vector2D(-y/l, x/l);
    }

    //-----------------------------------------------------------------------------
    //! @brief Computes the distance of two vectors
    //!
    //! @param[in]  b   The vector to which the distance should be computed
    //! @return         The distance of both vectors
    //-----------------------------------------------------------------------------
    inline double getDistanceTo(const Vector2D &b)
    {
        double dx = b.x - x;
        double dy = b.y - y;

        return sqrt(dx*dx + dy*dy);
    }

    //-----------------------------------------------------------------------------
    //! @brief Converts a vector into an OpenCV integer point
    //!
    //! @return The OpenCV integer point
    //-----------------------------------------------------------------------------
    inline Point toCvPoint()
    {
        return cv::Point((int) x, (int) y);
    }

    //-----------------------------------------------------------------------------
    //! @brief Converts a vector into an OpenCV floating point point
    //!
    //! @return Then OpenCV floating point point
    //-----------------------------------------------------------------------------
    inline CvPoint2D32f toCvPoint2D32f()
    {
        return cvPoint2D32f((float) x, (float) y);
    }
};
#endif

#endif /* #ifndef __MYTYPES_H */
