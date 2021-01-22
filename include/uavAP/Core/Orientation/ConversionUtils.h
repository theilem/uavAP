//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_CONVERSIONUTILS_H
#define UAVAP_CONVERSIONUTILS_H

#include "uavAP/Core/SensorData.h"

void
directionalConversion(FramedVector3& input, const Vector3& attitude, Frame target, Orientation orientation);

void
angularConversion(FramedVector3& angularRate, const Vector3& attitude, Frame target, Orientation orientation);

void
simpleFlipInertial(Vector3& input);

#endif //UAVAP_CONVERSIONUTILS_H
