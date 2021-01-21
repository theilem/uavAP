//
// Created by seedship on 1/21/21.
//

#ifndef UAVAP_CONVERSIONUTILS_H
#define UAVAP_CONVERSIONUTILS_H

#include "uavAP/Core/SensorData.h"


/**
 * Converts velocity or acceleration to inertial frame for ENU
 * @param input
 */
void
directionalToInertialENU(FramedVector3& input, const Vector3& attitude);

/**
 * Converts velocity or acceleration to inertial frame for NED
 * @param input
 */
void
directionalToInertialNED(FramedVector3& input, const Vector3& attitude);

/**
 * Converts velocity or acceleration to specified frame for ENU
 * @param input
 * @param attitude
 * @param target
 */
void
directionalToFrameENU(FramedVector3& input, const Vector3& attitude, Frame target);

/**
 * Converts velocity or acceleration to specified frame for NED
 * @param input
 * @param attitude
 * @param target
 */
void
directionalToFrameNED(FramedVector3& input, const Vector3& attitude, Frame target);

/**
 * Converts angular rate into inertial frame for ENU
 * (dot Phi, dot Theta, dot Psi)
 * @param angularRate
 * @param attitude
 */
void
angularToInertialENU(FramedVector3& angularRate, const Vector3& attitude);

/**
 * Converts inertial angular rate (dot phi, dot theta, dot psi) to body angular rate (pqr) for ENU
 * @param angularRate
 * @param attitude
 */
void
angularToBodyENU(FramedVector3& angularRate, const Vector3& attitude);

/**
 * Converts angular rate into inertial frame for NED
 * (dot Phi, dot Theta, dot Psi)
 * @param angularRate
 * @param attitude
 */
void
angularToInertialNED(FramedVector3& angularRate, const Vector3& attitude);

/**
 * Converts inertial angular rate (dot phi, dot theta, dot psi) to body angular rate (pqr) for NED
 * @param angularRate
 * @param attitude
 */
void
angularToBodyNED(FramedVector3& angularRate, const Vector3& attitude);

#endif //UAVAP_CONVERSIONUTILS_H
