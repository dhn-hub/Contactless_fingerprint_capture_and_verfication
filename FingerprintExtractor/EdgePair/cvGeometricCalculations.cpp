//*****************************************************************************
// Hilfsfunktionen f&uuml;r geometrische Berechnungen
//*****************************************************************************

#include <stdio.h>
#include <math.h>

#include "cvGeometricCalculations.h"

using namespace std;

//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden
//-----------------------------------------------------------------------------
double plDistance(Point* pointA, Point* pointB, Point* pointC)
{
    double eq[2][3];
    double valueA, valueB;

    eq[0][0] = pointB->x - pointA->x;
    eq[0][1] = (-1) * ( pointB->y - pointA->y );
    eq[0][2] = pointC->x - pointA->x;

    eq[1][0] = pointB->y - pointA->y;
    eq[1][1] = pointB->x - pointA->x;
    eq[1][2] = pointC->y - pointA->y;


    if(eq[0][0] != 0)
    {
        double factor = (-1) * eq[1][0] / eq[0][0];
        eq[1][0] += factor * eq[0][0];
        eq[1][1] += factor * eq[0][1];
        eq[1][2] += factor * eq[0][2];
    }
    else
    {
        double tempValue;

        tempValue = eq[0][0];
        eq[0][0] = eq[1][0];
        eq[1][0] = tempValue;

        tempValue = eq[0][1];
        eq[0][1] = eq[1][1];
        eq[1][1] = tempValue;

        tempValue = eq[0][2];
        eq[0][2] = eq[1][2];
        eq[1][2] = tempValue;
    }


    valueB = ( eq[1][2] / eq[1][1] );

    valueA = ( eq[0][2] - ( eq[0][1] * valueB) ) / eq[0][0];

    Point pointD = { (int)(pointA->x + ( valueA * ( pointB->x - pointA->x ) ) ), (int)(pointA->y + ( valueA * ( pointB->y - pointA->y ) ) )};

    double distance = sqrt( ( pow( (double)( pointD.x - pointC->x ),2 ) + pow( (double)( pointD.y - pointC->y ),2 )));

    return distance;
};


//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden
//-----------------------------------------------------------------------------
double plDistance(Point pointA, Point pointB, Point pointC)
{
    double eq[2][3];
    double valueA, valueB;

    eq[0][0] = pointB.x - pointA.x;
    eq[0][1] = (-1) * ( pointB.y - pointA.y );
    eq[0][2] = pointC.x - pointA.x;

    eq[1][0] = pointB.y - pointA.y;
    eq[1][1] = pointB.x - pointA.x;
    eq[1][2] = pointC.y - pointA.y;


    if(eq[0][0] != 0)
    {
        float factor = (-1) * eq[1][0] / eq[0][0];
        eq[1][0] += factor * eq[0][0];
        eq[1][1] += factor * eq[0][1];
        eq[1][2] += factor * eq[0][2];
    }
    else
    {
        float tempValue;

        tempValue = eq[0][0];
        eq[0][0] = eq[1][0];
        eq[1][0] = tempValue;

        tempValue = eq[0][1];
        eq[0][1] = eq[1][1];
        eq[1][1] = tempValue;

        tempValue = eq[0][2];
        eq[0][2] = eq[1][2];
        eq[1][2] = tempValue;
    }


    valueB = ( eq[1][2] / eq[1][1] );

    valueA = ( eq[0][2] - ( eq[0][1] * valueB) ) / eq[0][0];

    Point pointD = {(int)(pointA.x + ( valueA * ( pointB.x - pointA.x ) ) ), (int)(pointA.y + ( valueA * ( pointB.y - pointA.y ) ) )};

    double distance = sqrt( (double)( pow( (double)( pointD.x - pointC.x ),2 ) + pow( (double)( pointD.y - pointC.y ),2 )));

    return distance;
};


//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden
//-----------------------------------------------------------------------------
double plDistance(CvPoint2D32f pointA, CvPoint2D32f pointB, CvPoint2D32f pointC)
{
    double eq[2][3];
    double valueA, valueB;

    eq[0][0] = pointB.x - pointA.x;
    eq[0][1] = (-1) * ( pointB.y - pointA.y );
    eq[0][2] = pointC.x - pointA.x;

    eq[1][0] = pointB.y - pointA.y;
    eq[1][1] = pointB.x - pointA.x;
    eq[1][2] = pointC.y - pointA.y;


    if(eq[0][0] != 0)
    {
        float factor = (-1) * eq[1][0] / eq[0][0];
        eq[1][0] += factor * eq[0][0];
        eq[1][1] += factor * eq[0][1];
        eq[1][2] += factor * eq[0][2];
    }
    else
    {
        float tempValue;

        tempValue = eq[0][0];
        eq[0][0] = eq[1][0];
        eq[1][0] = tempValue;

        tempValue = eq[0][1];
        eq[0][1] = eq[1][1];
        eq[1][1] = tempValue;

        tempValue = eq[0][2];
        eq[0][2] = eq[1][2];
        eq[1][2] = tempValue;
    }


    valueB = ( eq[1][2] / eq[1][1] );

    valueA = ( eq[0][2] - ( eq[0][1] * valueB) ) / eq[0][0];

    Point pointD = {(int)(pointA.x + ( valueA * ( pointB.x - pointA.x ) ) ), (int)(pointA.y + ( valueA * ( pointB.y - pointA.y ) ) )};

    double distance = ( ( pow( ( pointD.x - pointC.x ),2 ) + pow( ( pointD.y - pointC.y ),2 )));

    distance = sqrt(distance);

    return distance;
};



//-----------------------------------------------------------------------------
//! Abstand von einem Punkt zu einer (durch zwei Punkte definierten) Geraden und
//! der Lage des Punkts relativ zur ihr (oberhalb/unterhalb)
//!
//! Punkte, die von der Gerade aus in Richtung ihrer Normalen liegen, werden als oberhalb der Gerade liegend
//! definiert. Die Normale entspricht dabei dem um 90&deg; gegen den Uhrzeigersinn gedrehten Richtungsvektor der Geraden.
//-----------------------------------------------------------------------------
double plDistanceSigned(Point* pointA, Point* pointB, Point* pointC)
{
    float eq[2][3];
    float valueA, valueB;

    eq[0][0] = pointB->x - pointA->x;
    eq[0][1] = (-1) * ( pointB->y - pointA->y );
    eq[0][2] = pointC->x - pointA->x;

    eq[1][0] = pointB->y - pointA->y;
    eq[1][1] = pointB->x - pointA->x;
    eq[1][2] = pointC->y - pointA->y;

    if(eq[0][0] != 0)
    {
        float factor = (-1) * eq[1][0] / eq[0][0];
        //cout << "factor = " << factor << endl;
        eq[1][0] += factor * eq[0][0];
        eq[1][1] += factor * eq[0][1];
        eq[1][2] += factor * eq[0][2];
    }
    else
    {
        float tempValue;

        tempValue = eq[0][0];
        eq[0][0] = eq[1][0];
        eq[1][0] = tempValue;

        tempValue = eq[0][1];
        eq[0][1] = eq[1][1];
        eq[1][1] = tempValue;

        tempValue = eq[0][2];
        eq[0][2] = eq[1][2];
        eq[1][2] = tempValue;
    }

    valueB = ( eq[1][2] / eq[1][1] );

    valueA = ( eq[0][2] - ( eq[0][1] * valueB) ) / eq[0][0];

    CvPoint2D32f pointD = { ( (float)pointA->x + ( valueA * ( (float)pointB->x - (float)pointA->x ) ) ),
                            ( (float)pointA->y + ( valueA * ( (float)pointB->y - (float)pointA->y ) ) )};

    double distance = sqrt( ( pow( ( pointD.x - (float)pointC->x ),2 ) + pow( ( pointD.y - (float)pointC->y ),2 )));

    if(valueB <= 0)
    {
        distance = ((-1)*distance);
    }

    return distance;

};



//-----------------------------------------------------------------------------
//! Berechnung des Schnittpunkts zweier Geraden
//-----------------------------------------------------------------------------
//int llIntersection(CvPoint pointA1, CvPoint pointA2, CvPoint pointB1, CvPoint pointB2, double& valueLineA, double& valueLineB)
int llIntersection(Vector2D pointA1, Vector2D pointA2, Vector2D pointB1, Vector2D pointB2, double& valueLineA, double& valueLineB)
{
    double eq[2][3];
    double valueA, valueB;

    eq[0][0] = pointA2.x - pointA1.x;
    eq[0][1] = (-1) * ( pointB2.x - pointB1.x );
    eq[0][2] = pointB1.x - pointA1.x;

    eq[1][0] = pointA2.y - pointA1.y;
    eq[1][1] = (-1) * ( pointB2.y - pointB1.y );
    eq[1][2] = pointB1.y - pointA1.y;

    if( ( eq[0][0] == eq[1][0] ) && ( eq[0][1] == eq[1][1] ) )
    {
        return -1;
    }
    else
    {
        if(eq[0][0] != 0)
        {
            float factor = (-1) * eq[1][0] / eq[0][0];
            eq[1][0] += factor * eq[0][0];
            eq[1][1] += factor * eq[0][1];
            eq[1][2] += factor * eq[0][2];
        }
        else
        {
            float tempValue;

            tempValue = eq[0][0];
            eq[0][0] = eq[1][0];
            eq[1][0] = tempValue;

            tempValue = eq[0][1];
            eq[0][1] = eq[1][1];
            eq[1][1] = tempValue;

            tempValue = eq[0][2];
            eq[0][2] = eq[1][2];
            eq[1][2] = tempValue;
        }

        if( eq[1][1] == 0 )
        {
            return 1;
        }
        else
        {
            valueB = ( eq[1][2] / eq[1][1] );
            valueLineB = valueB;

            valueA = ( eq[0][2] - ( eq[0][1] * valueB) ) / eq[0][0];
            valueLineA = valueA;
        }
    }
       return 0;
};



//-----------------------------------------------------------------------------
//! Berechnung des Schnittpunkts zweier Geraden
//-----------------------------------------------------------------------------
//int llIntersection(CvPoint* pointA1, CvPoint* pointA2, CvPoint* pointB1, CvPoint* pointB2, double& valueLineA, double& valueLineB)
int llIntersection(Vector2D* pointA1, Vector2D* pointA2, Vector2D* pointB1, Vector2D* pointB2, double& valueLineA, double& valueLineB)
{
    double eq[2][3];
    double valueA, valueB;

    eq[0][0] = pointA2->x - pointA1->x;
    eq[0][1] = (-1) * ( pointB2->x - pointB1->x );
    eq[0][2] = pointB1->x - pointA1->x;

    eq[1][0] = pointA2->y - pointA1->y;
    eq[1][1] = (-1) * ( pointB2->y - pointB1->y );
    eq[1][2] = pointB1->y - pointA1->y;

    if( ( eq[0][0] == eq[1][0] ) && ( eq[0][1] == eq[1][1] ) )
    {
        return -1;
    }
    else
    {
        if(eq[0][0] != 0)
        {
            float factor = (-1) * eq[1][0] / eq[0][0];
            eq[1][0] += factor * eq[0][0];
            eq[1][1] += factor * eq[0][1];
            eq[1][2] += factor * eq[0][2];
        }
        else
        {
            float tempValue;

            tempValue = eq[0][0];
            eq[0][0] = eq[1][0];
            eq[1][0] = tempValue;

            tempValue = eq[0][1];
            eq[0][1] = eq[1][1];
            eq[1][1] = tempValue;

            tempValue = eq[0][2];
            eq[0][2] = eq[1][2];
            eq[1][2] = tempValue;
        }

        if( eq[1][1] == 0 )
        {
            return 1;
        }
        else
        {
            valueB = ( eq[1][2] / eq[1][1] );
            valueLineB = valueB;

            valueA = ( eq[0][2] - ( eq[0][1] * valueB) ) / eq[0][0];
            valueLineA = valueA;
        }
    }
       return 0;
};


//-----------------------------------------------------------------------------
//! Berechnung des Winkels zwischen den Geraden A und B
//-----------------------------------------------------------------------------
double llAngle(Point* vectorA, Point* vectorB)
{
    double a1, angle;

    a1 = ( vectorA->x * vectorB->x ) + ( vectorA->y * vectorB->y );

    angle = acos( a1 / ( vLength( vectorA ) * vLength( vectorB ) ));

    return angle;
};


//-----------------------------------------------------------------------------
//! Berechnung des Winkels zwischen den Geraden A und B
//-----------------------------------------------------------------------------
double llAngle(Point vectorA, Point vectorB)
{
    double a1, angle;

    a1 = ( vectorA.x * vectorB.x ) + ( vectorA.y * vectorB.y );

    angle = acos( a1 / ( vLength( vectorA ) * vLength( vectorB ) ));

    return angle;
};



//-----------------------------------------------------------------------------
//! Berechnung des Winkels zwischen den Geraden A und B
//-----------------------------------------------------------------------------
double llAngle(CvPoint2D32f vectorA, CvPoint2D32f vectorB)
{
    double a1, angle;

    a1 = ( vectorA.x * vectorB.x ) + ( vectorA.y * vectorB.y );

    angle = acos( a1 / ( vLength( vectorA ) * vLength( vectorB ) ));

    return angle;
};



//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Liniensegments
//-----------------------------------------------------------------------------
double lLength(Point* pointA, Point* pointB)
{
    Point vectorA = {( pointA->x - pointB->x ), ( pointA->y - pointB->y ) };
    return vLength( vectorA );
};



//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Liniensegments
//-----------------------------------------------------------------------------
double lLength(Point pointA, Point pointB)
{
    Point vectorA = {( pointA.x - pointB.x ), ( pointA.y - pointB.y ) };
    return vLength( vectorA );
};



//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Liniensegments
//-----------------------------------------------------------------------------
double lLength(CvPoint2D32f pointA, CvPoint2D32f pointB)
{
    CvPoint2D32f vectorA = {( pointA.x - pointB.x ), ( pointA.y - pointB.y ) };
    return vLength( vectorA );
};



//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Vektors (euklidische Norm)
//-----------------------------------------------------------------------------
double vLength(Point vectorA)
{
    return sqrt( (double)( pow( (double)vectorA.x , 2) + pow( (double)vectorA.y , 2) ));
};



//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Vektors (euklidische Norm)
//-----------------------------------------------------------------------------
double vLength(Point* vectorA)
{
    return sqrt( (double) ( pow( (double)vectorA->x , 2) + pow( (double)vectorA->y , 2)) );
};



//-----------------------------------------------------------------------------
//! Berechnung der L&auml;nge eines Vektors (euklidische Norm)
//-----------------------------------------------------------------------------
double vLength(CvPoint2D32f vectorA)
{
        double x = pow( (double)vectorA.x , 2);
        double y = pow( (double)vectorA.y , 2);
        return sqrt((double)(x+y));
};



//-----------------------------------------------------------------------------
//! &Uuml;berpr&uuml;ft, ob ein Punkt innerhalb einer Bounding Box liegt
//-----------------------------------------------------------------------------
double PointInsideBBox( CvPoint2D32f* point, CvBox2D* bBox)
{

    CvPoint2D32f vTemp1 = { (float)(point->x - bBox->center.x), (float)(point->y - bBox->center.y)};
    CvPoint2D32f vTemp2 = { (float)cos(bBox->angle*CV_PI/180.0), (float)sin(bBox->angle*CV_PI/180.0)};

    double c = vLength(vTemp1);

    double alpha = llAngle(vTemp1, vTemp2);

    double beta = (CV_PI/2.0) - alpha;

    double a = (sin(alpha)*c) - (0.5*bBox->size.width);
    if( a < 0)
    {
        a = 0;
    }

    double b = (sin(beta)*c)- (0.5*bBox->size.height);

    if( b < 0)
    {
        b = 0;
    }

    double d = sqrt( (double)(pow( (double)a, 2 ) + pow((double) b, 2 ) ));

    return d;
};




//-----------------------------------------------------------------------------
//! &Uuml;berpr&uuml;ft, ob ein Punkt innerhalb einer Bounding Box liegt
//-----------------------------------------------------------------------------
double PointInsideBBox( Point* point, CvBox2D* bBox)
{

    CvPoint2D32f vTemp1 = { (float)(point->x - bBox->center.x), (float)(point->y - bBox->center.y)};
    CvPoint2D32f vTemp2 = { (float)cos(bBox->angle*CV_PI/180.0), (float)sin(bBox->angle*CV_PI/180.0)};

    double c = vLength(vTemp1);

    double alpha = llAngle(vTemp1, vTemp2);

    double beta = (CV_PI/2.0) - alpha;

    double a = (sin(alpha)*c) - (0.5*bBox->size.width);
    if( a < 0)
    {
        a = 0;
    }

    double b = (sin(beta)*c)- (0.5*bBox->size.height);

    if( b < 0)
    {
        b = 0;
    }

    double d = sqrt( (double)(pow( (double)a, 2 ) + (double)pow( b, 2 ) ));

    return d;
};

