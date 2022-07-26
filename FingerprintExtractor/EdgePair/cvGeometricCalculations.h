#ifndef GEOMETRICCALCULATIONS_H
#define GEOMETRICCALCULATIONS_H

#include "opencv2/opencv.hpp"
//#include "clsVector2d.h"
#include "myTypes.h"

using namespace cv;
//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden
//!
//! @param[in]     pointA   erster Punkte auf der Geraden
//! @param[in]     pointB   erster Punkte auf der Geraden
//! @param[in]     pointC   Punkt
//! @return                 Abstand (Pixel)
//-----------------------------------------------------------------------------
double plDistance(Point* pointA, Point* pointB, Point* pointC);


//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden
//!
//! @param[in]     pointA   erster Punkte auf der Geraden
//! @param[in]     pointB   erster Punkte auf der Geraden
//! @param[in]     pointC   Punkt
//! @return                 Abstand (Pixel)
//-----------------------------------------------------------------------------
double plDistance(Point pointA, Point pointB, Point pointC);


//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden
//!
//! @param[in]     pointA   erster Punkte auf der Geraden
//! @param[in]     pointB   erster Punkte auf der Geraden
//! @param[in]     pointC   Punkt
//! @return                 Abstand (Pixel)
//-----------------------------------------------------------------------------
double plDistance(CvPoint2D32f pointA, CvPoint2D32f pointB, CvPoint2D32f pointC);


//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden und
//! der Lage des Punkts relativ zur ihr (oberhalb/unterhalb)
//!
//! @param[in]     pointA   erster Punkte auf der Geraden
//! @param[in]     pointB   erster Punkte auf der Geraden
//! @param[in]     pointC   Punkt
//! @return                 Abstand;
//! @return                 negativer Wert, wenn Punkt unterhalb der Geraden
//! @return                 positiver Wert, wenn Punkt oberhalb der Geraden
//-----------------------------------------------------------------------------
double plDistanceSigned(Point* pointA, Point* pointB, Point* pointC);


//-----------------------------------------------------------------------------
//! Berechnung des Schnittpunkts zweier Geraden
//!
//! @param[in]      pointA1         erster Punkt auf Gerade A
//! @param[in]      pointA2         zweiter Punkt auf Gerade A
//! @param[in]      pointB1         erster Punkt auf Gerade B
//! @param[in]      pointB2         zweiter Punkt auf Gerade B
//! @param[out]     valueLineA      Wert x fu&ml;r den pointA1 + x * (pointA2-pointA1)
//!                                 den Schnittpunkt ergibt
//! @param[out]     valueLineB      Wert x fu&ml;r den pointB1 + x * (pointB2-pointB1)
//!                                 den Schnittpunkt ergibt
//! @return                         0, falls ein Schnittpunkt gefunden wird
//! @return                         1, sonst
//-----------------------------------------------------------------------------
//int llIntersection(CvPoint pointA1, CvPoint pointA2, CvPoint pointB1, CvPoint pointB2, double& valueLineA, double& valueLineB);
int llIntersection(Vector2D pointA1, Vector2D pointA2, Vector2D pointB1, Vector2D pointB2, double& valueLineA, double& valueLineB);

//-----------------------------------------------------------------------------
//! Berechnung des Schnittpunkts zweier Geraden
//!
//! @param[in]      pointA1         erster Punkt auf Gerade A
//! @param[in]      pointA2         zweiter Punkt auf Gerade A
//! @param[in]      pointB1         erster Punkt auf Gerade B
//! @param[in]      pointB2         zweiter Punkt auf Gerade B
//! @param[out]     valueLineA      Wert x fuer den pointA1 + x * (pointA2-pointA1)
//!                                 den Schnittpunkt ergibt
//! @param[out]     valueLineB      Wert x fuer den pointB1 + x * (pointB2-pointB1)
//!                                 den Schnittpunkt ergibt
//! @return                         0, falls ein Schnittpunkt gefunden wird
//! @return                         1, sonst
//-----------------------------------------------------------------------------
//int llIntersection(CvPoint* pointA1, CvPoint* pointA2, CvPoint* pointB1, CvPoint* pointB2, double& valueLineA, double& valueLineB);
int llIntersection(Vector2D* pointA1, Vector2D* pointA2, Vector2D* pointB1, Vector2D* pointB2, double& valueLineA, double& valueLineB);

//-----------------------------------------------------------------------------
//! Berechnung des Winkels zwischen den Geraden A und B
//!
//! @param[in]     vectorA  Richtungsvektor Gerade A
//! @param[in]     vectorB  Richtungsvektor Gerade B
//! @return                 Winkel im Bogenma&szlig;
//-----------------------------------------------------------------------------
double llAngle(Point* vectorA, Point* vectorB);


//-----------------------------------------------------------------------------
//! Berechnung des Winkels zwischen den Geraden A und B
//!
//! @param[in]     vectorA  Richtungsvektor Gerade A
//! @param[in]     vectorB  Richtungsvektor Gerade B
//! @return                 Winkel im Bogenma&szlig;
//-----------------------------------------------------------------------------
double llAngle(Point vectorA, Point vectorB);


//-----------------------------------------------------------------------------
//! Berechnung des Winkels zwischen den Geraden A und B
//!
//! @param[in]     vectorA  Richtungsvektor Gerade A
//! @param[in]     vectorB  Richtungsvektor Gerade B
//! @return                 Winkel im Bogenma&szlig;
//-----------------------------------------------------------------------------
double llAngle(CvPoint2D32f vectorA, CvPoint2D32f vectorB);


//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Liniensegments
//!
//! @param[in]     pointA   Startpunkt
//! @param[in]     pointB   Endpunkt
//! @return                 L&auml;nge (Pixel)
//-----------------------------------------------------------------------------
double lLength(Point* pointA, Point* pointB);


//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Liniensegments
//!
//! @param[in]     pointA   Startpunkt
//! @param[in]     pointB   Endpunkt
//! @return                 L&auml;nge (Pixel)
//-----------------------------------------------------------------------------
double lLength(Point pointA, Point pointB);


//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Liniensegments
//!
//! @param[in]     pointA   Startpunkt
//! @param[in]     pointB   Endpunkt
//! @return                 L&auml;nge (Pixel)
//-----------------------------------------------------------------------------
double lLength(CvPoint2D32f pointA, CvPoint2D32f pointB);


//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Vektors (euklidische Norm)
//!
//! @param[in]     vectorA  Vektor
//! @return                 L&auml;nge (Pixel)
//-----------------------------------------------------------------------------
double vLength(Point* vectorA);


//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Vektors (euklidische Norm)
//!
//! @param[in]     vectorA  Vektor
//! @return                 L&auml;nge (Pixel)
//-----------------------------------------------------------------------------
double vLength(Point vectorA);


//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Vektors (euklidische Norm)
//!
//! @param[in]     vectorA  Vektor
//! @return                 L&auml;nge (Pixel)
//-----------------------------------------------------------------------------
double vLength(CvPoint2D32f vectorA);


//-----------------------------------------------------------------------------
//! &Uuml;berpr&uuml;ft, ob ein Punkt innerhalb einer Bounding Box liegt
//!
//! @param[in]     point    Punkt
//! @param[in]     bBox     Bounding Box
//! @return                 0, falls der Punkt auf oder innerhalb der Bounding Box liegt
//! @return                 Abstand des Punkts zur Bounding Box, sonst
//-----------------------------------------------------------------------------
double PointInsideBBox( CvPoint2D32f* point, CvBox2D* bBox);


//-----------------------------------------------------------------------------
//! &Uuml;berpr&uuml;ft, ob ein Punkt innerhalb einer Bounding Box liegt
//!
//! @param[in]     point    Punkt
//! @param[in]     bBox     Bounding Box
//! @return                 0, falls der Punkt auf oder innerhalb der Bounding Box liegt
//! @return                 Abstand des Punkts zur Bounding Box, sonst
//-----------------------------------------------------------------------------
double PointInsideBBox( Point* point, CvBox2D* bBox);


#endif // GEOMETRICCALCULATIONS_H
