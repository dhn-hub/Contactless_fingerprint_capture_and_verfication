//*****************************************************************************
//! @file  CvPointOperations.h
//! @brief Interface-Datei fuer CvPointOperations.cpp
//*****************************************************************************

#ifndef CVPOINTOPERATIONS_H
#define CVPOINTOPERATIONS_H

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

//! Erweiterung zu CvPoint mit Standard-Methoden
class CvPointOPs : public Point
{
public:

    //-----------------------------------------------------------------------------
    //! @brief Konstruktor
    //!
    //! @param[in]     -
    //! @return        initialer PUnkt (0,0)
    //-----------------------------------------------------------------------------
    CvPointOPs();

    //-----------------------------------------------------------------------------
    //! @brief Konstruktor
    //!
    //! @param[in]     a    x- und y-Koordinate
    //! @return        initialer PUnkt (a,a)
    //-----------------------------------------------------------------------------
    CvPointOPs(int a);

    //-----------------------------------------------------------------------------
    //! @brief Konstruktor
    //!
    //! @param[in]     a    x-Koordinate
    //! @param[in]     b    y-Koordinate
    //! @return        initialer PUnkt (a,b)
    //-----------------------------------------------------------------------------
    CvPointOPs(int a, int b);

    //-----------------------------------------------------------------------------
    //! @brief Konstruktor
    //!
    //! @param[in]     p    Punkt
    //! @return        initialer PUnkt
    //-----------------------------------------------------------------------------
    CvPointOPs(Point p);


    //-----------------------------------------------------------------------------
    //! @brief Kopier-Konstruktor
    //!
    //! @param[in]     rhs      Referenz-Punkt
    //! @return        initialer PUnkt
    //-----------------------------------------------------------------------------
    CvPointOPs(const CvPointOPs& rhs); // copy constructor

    //-----------------------------------------------------------------------------
    //! @brief Destruktor
    //!
    //! @param[in]     -
    //! @return        -
    //-----------------------------------------------------------------------------
    ~CvPointOPs();

    //binaere Operatoren

    CvPointOPs& operator= (const Point&); //!< Zuweisungsoperator
    CvPointOPs operator+ (const Point&);  //!< Operator +
    CvPointOPs operator- (const Point&);  //!< Operator -
    CvPointOPs& operator+=(const CvPointOPs&);   //!< Operator +=
    CvPointOPs& operator-=(const CvPointOPs&);   //!< Operator -=
    CvPointOPs operator* (const int&);  //!< Operator * (Skalierung mit int)
    CvPointOPs operator/ (const int&);  //!< Operator / (Skalierung mit int)

    //Vergleichsoperatoren
    bool operator== (const Point&) const; //!< Operator == (Koordinatengleichheit)
    bool operator== (const int&) const; //!< Operator == (Laengenvergleich mit int)
    bool operator== (const double&) const; //!< Operator == (Laengenvergleich double)
    bool operator!= (const Point&) const; //!< Operator != (Koordinatengleichheit)
    bool operator!= (const int&) const; //!< Operator != (Laengenvergleich mit int)
    bool operator!= (const double&) const; //!< Operator != (Laengenvergleich double)
    bool operator > (const CvPointOPs&) const; //!< Operator > (Betragsvergleich)
    bool operator < (const CvPointOPs&) const; //!< Operator > (Betragsvergleich)

    //-----------------------------------------------------------------------------
    //! @brief bildet den Absolutbetrag vom Ursprung bis zum Punkt
    //!
    //! @param[in]     -
    //! @return        double
    //-----------------------------------------------------------------------------
    double abs();

    double dist(CvPointOPs dest);

    //insertion redirection:
    //    bool operator== (const float&) const;
    friend ostream& operator<<(ostream& out, const CvPointOPs&); //!<Ausgabeoperator
    //so geht's nicht:
    //     ostream& operator<<(ostream& out);

    //-----------------------------------------------------------------------------
    //! @brief prueft, ob ein Punkt innerhalb eines (gedrehten) Rechtecks liegt
    //!
    //! @param[in]     p        zu pruefender Punkt
    //! @param[in]     box      Rechteck
    //! @return        true oder false
    //-----------------------------------------------------------------------------
    static bool isPointInBox(Point p, CvBox2D box);
};


#endif // CVPOINTOPERATIONS_H
