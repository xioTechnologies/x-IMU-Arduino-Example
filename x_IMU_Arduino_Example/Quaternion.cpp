/*
    Quaternion.cpp
    Author: Seb Madgwick

    C++ library for basic usage of quaternions.
    See: http://www.x-io.co.uk/quaternions/
*/

//------------------------------------------------------------------------------
// Includes

#include "Quaternion.h"
#include <math.h>

//------------------------------------------------------------------------------
// Variables

float q[4];

//------------------------------------------------------------------------------
// Methods

Quaternion::Quaternion(void) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
}

Quaternion::Quaternion(const float w, const float x, const float y, const float z) {
    q[0] = w;
    q[1] = x;
    q[2] = y;
    q[3] = z;
}

Quaternion Quaternion::getConjugate(void) const {
    Quaternion conjugate;
    conjugate.q[0] = q[0];
    conjugate.q[1] = -q[1];
    conjugate.q[2] = -q[2];
    conjugate.q[3] = -q[3];
    return conjugate;
}

EulerAnglesStruct Quaternion::getEulerAngles(void) const {
    EulerAnglesStruct eulerAnglesStruct;
    eulerAnglesStruct.roll = radiansToDegrees(atan2(2.0f * (q[2] * q[3] - q[0] * q[1]), 2.0f * q[0] * q[0] - 1.0f + 2.0f * q[3] * q[3]));
    eulerAnglesStruct.pitch = radiansToDegrees(-atan((2.0f * (q[1] * q[3] + q[0] * q[2])) / sqrt(1.0f - pow((2.0f * q[1] * q[3] + 2.0f * q[0] * q[2]), 2.0f))));
    eulerAnglesStruct.yaw = radiansToDegrees(atan2(2.0f * (q[1] * q[2] - q[0] * q[3]), 2.0f * q[0] * q[0] - 1.0f + 2.0f * q[1] * q[1]));
    return eulerAnglesStruct;
}

float Quaternion::radiansToDegrees (float radians) const {
    return 57.2957795130823f * radians;
}

//------------------------------------------------------------------------------
// End of file
