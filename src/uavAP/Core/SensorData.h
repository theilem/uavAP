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
#include "uavAP/Core/EnumMap.hpp"

enum class Sensor
{
	INVALID,
	POSITION_X,
	POSITION_Y,
	POSITION_Z,
	VELOCITY_X,
	VELOCITY_Y,
	VELOCITY_Z,
	ACCELERATION_X,
	ACCELERATION_Y,
	ACCELERATION_Z,
	ATTITUDE_X,
	ATTITUDE_Y,
	ATTITUDE_Z,
	ANGULAR_RATE_X,
	ANGULAR_RATE_Y,
	ANGULAR_RATE_Z,
	TIMESTAMP,
	SEQUENCE_NR,
	HAS_GPS_FIX,
	AUTOPILOT_ACTIVE,
	AIR_SPEED,
	GROUND_SPEED,
	ANGLE_OF_ATTACK,
	PROPULSION_POWER,
	CONSUMED_ENERGY,
	BATTERY_VOLTAGE,
	BATTERY_CURRENT,
	AILERON,
	ELEVATOR,
	RUDDER,
	THROTTLE,
	RPM,
	NUM_SENSOR
};

ENUMMAP_INIT(Sensor,
	{
		{Sensor::POSITION_X, "position_x"},
		{Sensor::POSITION_Y, "position_y"},
		{Sensor::POSITION_Z, "position_z"},
		{Sensor::VELOCITY_X, "velocity_x"},
		{Sensor::VELOCITY_Y, "velocity_y"},
		{Sensor::VELOCITY_Z, "velocity_z"},
		{Sensor::ACCELERATION_X, "acceleration_x"},
		{Sensor::ACCELERATION_Y, "acceleration_y"},
		{Sensor::ACCELERATION_Z, "acceleration_z"},
		{Sensor::ATTITUDE_X, "attitude_x"},
		{Sensor::ATTITUDE_Y, "attitude_y"},
		{Sensor::ATTITUDE_Z, "attitude_z"},
		{Sensor::ANGULAR_RATE_X, "angular_rate_x"},
		{Sensor::ANGULAR_RATE_Y, "angular_rate_y"},
		{Sensor::ANGULAR_RATE_Z, "angular_rate_z"},
		{Sensor::TIMESTAMP, "timestamp"},
		{Sensor::SEQUENCE_NR, "sequence_nr"},
		{Sensor::HAS_GPS_FIX, "has_gps_fix"},
		{Sensor::AUTOPILOT_ACTIVE, "autopilot_active"},
		{Sensor::AIR_SPEED, "air_speed"},
		{Sensor::GROUND_SPEED, "ground_speed"},
		{Sensor::ANGLE_OF_ATTACK, "angle_of_attack"},
		{Sensor::PROPULSION_POWER, "propulsion_power"},
		{Sensor::CONSUMED_ENERGY, "consumed_energy"},
		{Sensor::BATTERY_VOLTAGE, "battery_voltage"},
		{Sensor::BATTERY_CURRENT, "battery_current"},
		{Sensor::AILERON, "aileron"},
		{Sensor::ELEVATOR, "elevator"},
		{Sensor::RUDDER, "rudder"},
		{Sensor::THROTTLE, "throttle"},
		{Sensor::RPM, "rpm"}
	}
);

/**
 * @brief SensorData struct. Contains everything needed for calculation of control.
 */
struct SensorData
{
	Vector3 position; 		//!< UTM Frame, [X: East, Y: North, Z: Up]
	Vector3 velocity; 		//!< Earth Frame
	Vector3 acceleration; 	//!< Body Frame
	Vector3 attitude; 		//!< [X: Roll, Y: Pitch, Z: Yaw]
	Vector3 angularRate; 	//!< [X: Roll, Y: Pitch, Z: Yaw]
	TimePoint timestamp; 	//!< Timestamp of sensor data, Format: [2019-Mar-13 18:21:21.354043]
	uint32_t sequenceNr; 	//!< Sequence number of the struct
	bool hasGPSFix; 		//!< Shows whether the GPS has a fix
	bool autopilotActive; 	//!< Shows if the autopilot is active, always true in simulation
	double airSpeed; 		//!< total velocity w.r.t. wind
	double groundSpeed; 	//!< total velocity w.r.t. ground
	double angleOfAttack; 	//!< current angle of attack
	double propulsionPower; //!< measured or estimated current propulsion power
	double consumedEnergy; 	//!< measured or estimated total used energy for propulsion
	double batteryVoltage;	//!< current battery voltage
	double batteryCurrent;	//!< current battery current
	double aileron;			//!< current aileron position
	double elevator;		//!< current elevator position
	double rudder;			//!< current rudder position
	double throttle;		//!< current throttle position
	double rpm;				//!< current motor rotation speed

	SensorData() :
				position(0, 0, 0), velocity(0, 0, 0), acceleration(0, 0, 0), attitude(0, 0, 0), angularRate(
						0, 0, 0), sequenceNr(0), hasGPSFix(false), autopilotActive(false), airSpeed(0), groundSpeed(
						0), angleOfAttack(0), propulsionPower(0), consumedEnergy(0), batteryVoltage(0), batteryCurrent(
						0), aileron(0), elevator(0), rudder(0), throttle(0), rpm(0)
	{
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
	ar & t.aileron;
	ar & t.elevator;
	ar & t.rudder;
	ar & t.throttle;
	ar & t.rpm;
}
}

#endif /* UAVAP_CORE_SENSORDATA_H_ */
