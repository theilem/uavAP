////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/**
 * @file SensorData.h
 * @brief SensorData and SensorDataLight definitions
 *
 * Defines SensorData and SensorDataLight and provides conversions between float and double datatypes
 * @date 23 June 2017
 * @author Mirco Theile, mircot@illinois.edu
 */
#ifndef UAVAP_CORE_SENSORDATA_H_
#define UAVAP_CORE_SENSORDATA_H_

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Time.h"

/**
 * @brief SensorData struct. Contains everything needed for calculation of control.
 */
struct SensorData
{
	Vector3 position; //!< UTM Frame, [X: North, Y: East, Z: Down]
	Vector3 velocity; //!< Earth Frame
	Vector3 acceleration; //!< Body Frame
	Vector3 attitude; //!< [X: Roll, Y: Pitch, Z: Yaw]
	Vector3 angularRate; //!< [X: Roll, Y: Pitch, Z: Yaw]
	Vector3 angularAcc;  //!< [X: Roll, Y: Pitch, Z: Yaw]
	TimePoint timestamp; //!< Timestamp of this sensor data struct
	double velocityAir; //!< total forward velocity w.r.t. wind
	double velocityGround; //!< total forward velocity w.r.t. ground
	bool hasGPSFix; //!< Shows whether the GPS has a fix
	bool autopilotActive; //!< Shows if the autopilot is active, always true in simulation

	double angleOfAttack; //!< current angle of attack

	double propulsionPower; //!< measured or estimated current propulsion power
	double consumedEnergy; //!< measured or estimated total used energy for propulsion
	uint32_t sequenceNr; //!< Sequence number of the struct

	SensorData() :
			position(0, 0, 0), velocity(0, 0, 0), acceleration(0, 0, 0), attitude(0, 0, 0), angularRate(
					0, 0, 0), angularAcc(0, 0, 0), velocityAir(0), velocityGround(0), hasGPSFix(
					false), autopilotActive(false), angleOfAttack(0), propulsionPower(0), consumedEnergy(
					0), sequenceNr(0)
	{
	}
};

/**
 * @brief SensorDataLight struct. Smaller version of the SensorData struct.
 *
 * Contains similar information as SensorData with only float32 precision. Booleans are
 * compressed in one 8bit integer.
 */
struct SensorDataLight
{
	using Vector3f = Eigen::Vector3f;
	Vector3f position; //!< UTM Frame, [X: North, Y: East, Z: Down]
	Vector3f velocity; //!< Velocity in Earth Frame
	Vector3f acceleration; //!< Acceleration Body Frame
	Vector3f attitude; //!< Attitude [X: Roll, Y: Pitch, Z: Yaw]
	Vector3f angularRate; //!< Angular rate [X: Roll, Y: Pitch, Z: Yaw]
	TimePoint timestamp; //!< Timestamp of this sensor data struct
	float velocityAir; //!< total forward velocity w.r.t. wind
	float velocityGround; //!< total forward velocity w.r.t. ground
	float angleOfAttack; //!< current angle of attack
	float propulsionPower; //!< measured or estimated current propulsion power
	float consumedEnergy; //!< measured or estimated total used energy for propulsion

	uint8_t flags; //!< Bit field containing:  [0,0,0,0,0,0,hasGPSFix,autopilotActive]

	uint32_t sequenceNr; //!< Sequence number of the struct
};

/**
 * @brief Convert float Vector3 to double
 * @param vec float Eigen Vector3
 * @return double Eigen Vector3
 */
Eigen::Vector3d
vectorFloatToDouble(const Eigen::Vector3f& vec);

/**
 * @brief Convert double Vector3 to float
 * @param vec double Eigen Vector3
 * @return float Eigen Vector3
 */
Eigen::Vector3f
vectorDoubleToFloat(const Eigen::Vector3d& vec);

/**
 * @brief Convert SensorDataLight to SensorData
 * @param sd SensorDataLight object to be converted
 * @return SensorData object
 */
SensorData
fromSensorDataLight(const SensorDataLight& sd);

/**
 * @brief Convert SensorData to SensorDataLight
 * @param sd SensorDataLight object to be converted
 * @return SensorDataLight object
 */
SensorDataLight
fromSensorData(const SensorData& sd);

#endif /* UAVAP_CORE_SENSORDATA_H_ */
