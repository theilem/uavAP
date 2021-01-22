//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_CONVERSIONUTILS_H
#define UAVAP_CONVERSIONUTILS_H

#include "uavAP/Core/SensorData.h"

/**
 * Converts velocity and acceleration vectors between different frames of the same orientation
 * @param direction
 * @param attitude
 * @param target
 * @param orientation
 */
void
directionalConversion(FramedVector3& direction, const Vector3& attitude, Frame target, Orientation orientation);

/**
 * Converts angular rate vectors between body frame (pqr) and inertial frame(dot{roll, pitch, yaw})
 * or dot{pitch, roll, yaw} of the same orientation
 * @param direction
 * @param attitude
 * @param target
 * @param orientation
 */
void
angularConversion(FramedVector3& angularRate, const Vector3& attitude, Frame target, Orientation orientation);

///**
// * Idempotently Flips angular rate orientation from ENU <-> NED amd keeps frame
// * @param input
// */
//void
//convertAngularOrientation(FramedVector3& angularRate, const Vector3& attitude);
//
///**
// * Idempotently Flips velocity/acceleration orientation from ENU <-> NED amd keeps frame
// * @param input
// */
//void
//convertDirectionalOrientation(FramedVector3& direction, const Vector3& attitude);


/**
 * Flips index 0 and 1, then negates index 2
 * @param input
 */
void
simpleFlipInertial(Vector3& input);

#endif //UAVAP_CONVERSIONUTILS_H
