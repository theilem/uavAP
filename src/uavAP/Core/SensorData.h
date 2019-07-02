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
 * @author Mirco Theile, mirco.theile@tum.de
 */
#ifndef UAVAP_CORE_SENSORDATA_H_
#define UAVAP_CORE_SENSORDATA_H_

#include "uavAP/Core/Frames/IFrame.h"
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Time.h"

/**
 * @brief SensorData struct. Contains everything needed for calculation of control.
 */
struct SensorData
{
	Vector3 position; //!< UTM Frame, [X: East, Y: North, Z: Up]
	Vector3 velocity; //!< Earth Frame
	Vector3 acceleration; //!< Body Frame
	Vector3 attitude; //!< [X: Roll, Y: Pitch, Z: Yaw]
	Vector3 angularRate; //!< [X: Roll, Y: Pitch, Z: Yaw]
	TimePoint timestamp; //!< Timestamp of this sensor data struct
	FloatingType airSpeed; //!< total velocity w.r.t. wind
	FloatingType groundSpeed; //!< total velocity w.r.t. ground
	bool hasGPSFix; //!< Shows whether the GPS has a fix
	bool autopilotActive; //!< Shows if the autopilot is active, always true in simulation

	FloatingType angleOfAttack; //!< current angle of attack

	FloatingType propulsionPower; //!< measured or estimated current propulsion power
	FloatingType consumedEnergy; //!< measured or estimated total used energy for propulsion
	uint32_t sequenceNr; //!< Sequence number of the struct

	FloatingType batteryVoltage;
	FloatingType batteryCurrent;
	FloatingType throttle;
	FloatingType rpm;

	inline
	SensorData() :
			position(0, 0, 0), velocity(0, 0, 0), acceleration(0, 0, 0), attitude(0, 0, 0), angularRate(
					0, 0, 0), airSpeed(0), groundSpeed(0), hasGPSFix(false), autopilotActive(false), angleOfAttack(
					0), propulsionPower(0), consumedEnergy(0), sequenceNr(0), batteryVoltage(0), batteryCurrent(
					0), throttle(0), rpm(0)
	{
	}

	inline
	SensorData(const SensorData& other) :
			position(other.position), velocity(other.velocity), acceleration(other.acceleration), attitude(
					other.attitude), angularRate(other.angularRate), timestamp(other.timestamp), airSpeed(
					other.airSpeed), groundSpeed(other.groundSpeed), hasGPSFix(other.hasGPSFix), autopilotActive(
					other.autopilotActive), angleOfAttack(other.angleOfAttack), propulsionPower(
					other.propulsionPower), consumedEnergy(other.consumedEnergy), sequenceNr(
					other.sequenceNr), batteryVoltage(other.batteryVoltage), batteryCurrent(
					other.batteryCurrent), throttle(other.throttle), rpm(other.rpm)
	{
	}

	inline SensorData&
	operator=(const SensorData& other)
	{
		position = other.position;
		velocity = other.velocity;
		acceleration = other.acceleration;
		attitude = other.attitude;
		angularRate = other.angularRate;
		timestamp = other.timestamp;
		airSpeed = other.airSpeed;
		groundSpeed = other.groundSpeed;
		hasGPSFix = other.hasGPSFix;
		autopilotActive = other.autopilotActive;
		angleOfAttack = other.angleOfAttack;
		propulsionPower = other.propulsionPower;
		consumedEnergy = other.consumedEnergy;
		sequenceNr = other.sequenceNr;
		batteryVoltage = other.batteryVoltage;
		batteryCurrent = other.batteryCurrent;
		throttle = other.throttle;
		rpm = other.rpm;
		return *this;
	}
};

SensorData&
changeFrame(const IFrame& orig, const IFrame& dest, SensorData& data);

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, SensorData& t)
{
	ar & t.position;
	ar & t.velocity;
	ar & t.acceleration;
	ar & t.attitude;
	ar & t.angularRate;
	ar & t.timestamp;
	ar & t.airSpeed;
	ar & t.groundSpeed;
	ar & t.propulsionPower;
	ar & t.consumedEnergy;
	ar & t.hasGPSFix;
	ar & t.autopilotActive;
	ar & t.angleOfAttack;
	ar & t.sequenceNr;
	ar & t.batteryVoltage;
	ar & t.batteryCurrent;
	ar & t.throttle;
	ar & t.rpm;
}
}

#endif /* UAVAP_CORE_SENSORDATA_H_ */
