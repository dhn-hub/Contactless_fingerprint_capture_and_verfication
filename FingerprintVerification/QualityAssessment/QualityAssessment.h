#ifndef QUALTYASSESMENT_H
#define QUALTYASSESMENT_H

#include <opencv2/opencv.hpp>

#include "sobelEdgeStrength.h"
#include "SFPQA.h"


class QualityAssessment
{
public:
    // Construtor
    QualityAssessment();

    // get sharpness
    SFPQA sharpnessSobelGrad(SFinger finger);

    std::vector<int> sortDesc(std::vector<float> unsortedList);

private:    
    //std::vector<SFPQA> mFingerList; // TODO

};

#endif // QUALTYASSESMENT_H
