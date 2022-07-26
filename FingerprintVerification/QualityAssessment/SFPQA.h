#ifndef SFPQA_H
#define SFPQA_H

#include "SFinger.h"


struct SFPQA
{
    SFPQA()
    {
        mSharpnessContrastSobelGrad = 0.0;
        isValid = false;
    }


    SFPQA& operator=(const SFPQA& other)
    {
        mFinger = other.mFinger;
        isValid = other.isValid;
        mSharpnessContrastSobelGrad = other.mSharpnessContrastSobelGrad;
        return *this;
    }

    ~SFPQA(){}

    bool isValid;
    SFinger mFinger;
    float mSharpnessContrastSobelGrad;
};

#endif // SFPQA_H
