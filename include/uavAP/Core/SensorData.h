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
 * @file SensorEnumData.h
 * @brief SensorEnumData definitions
 *
 * @date 23 June 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */
#ifndef UAVAP_CORE_SensorEnumDATA_H_
#define UAVAP_CORE_SensorEnumDATA_H_

#include "uavAP/Core/Frames/IFrame.h"
#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/Time.hpp>
#include <cpsCore/Utilities/EnumMap.hpp>
#include <cpsCore/Utilities/DataPresentation/detail/Split.h>

enum class SensorEnum
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
	ANGLE_OF_SIDESLIP,
	NUM_SENSOR_ENUM
};

enum class PowerEnum
{
	PROPULSION_POWER,
	CONSUMED_ENERGY,
	BATTERY_VOLTAGE,
	BATTERY_CURRENT,
	NUM_POWER_ENUM
};

enum class ServoEnum
{
	AILERON,
	ELEVATOR,
	RUDDER,
	THROTTLE,
	RPM,
	NUM_SERVO_ENUM
};

ENUMMAP_INIT(SensorEnum,
			 {
				 { SensorEnum::POSITION_X, "position_x" },
				 { SensorEnum::POSITION_Y, "position_y" },
				 { SensorEnum::POSITION_Z, "position_z" },
				 { SensorEnum::VELOCITY_X, "velocity_x" },
				 { SensorEnum::VELOCITY_Y, "velocity_y" },
				 { SensorEnum::VELOCITY_Z, "velocity_z" },
				 { SensorEnum::ACCELERATION_X, "acceleration_x" },
				 { SensorEnum::ACCELERATION_Y, "acceleration_y" },
				 { SensorEnum::ACCELERATION_Z, "acceleration_z" },
				 { SensorEnum::ATTITUDE_X, "attitude_x" },
				 { SensorEnum::ATTITUDE_Y, "attitude_y" },
				 { SensorEnum::ATTITUDE_Z, "attitude_z" },
				 { SensorEnum::ANGULAR_RATE_X, "angular_rate_x" },
				 { SensorEnum::ANGULAR_RATE_Y, "angular_rate_y" },
				 { SensorEnum::ANGULAR_RATE_Z, "angular_rate_z" },
				 { SensorEnum::TIMESTAMP, "timestamp" },
				 { SensorEnum::SEQUENCE_NR, "sequence_nr" },
				 { SensorEnum::HAS_GPS_FIX, "has_gps_fix" },
				 { SensorEnum::AUTOPILOT_ACTIVE, "autopilot_active" },
				 { SensorEnum::AIR_SPEED, "air_speed" },
				 { SensorEnum::GROUND_SPEED, "ground_speed" },
				 { SensorEnum::ANGLE_OF_ATTACK, "angle_of_attack" },
				 { SensorEnum::ANGLE_OF_SIDESLIP, "angle_of_sideslip" }
			 }
);


ENUMMAP_INIT(PowerEnum,
			 {
				 { PowerEnum::PROPULSION_POWER, "propulsion_power" },
				 { PowerEnum::CONSUMED_ENERGY, "consumed_energy" },
				 { PowerEnum::BATTERY_VOLTAGE, "battery_voltage" },
				 { PowerEnum::BATTERY_CURRENT, "battery_current" }
			 }
);

ENUMMAP_INIT(ServoEnum,
			 {
				 { ServoEnum::AILERON, "aileron" },
				 { ServoEnum::ELEVATOR, "elevator" },
				 { ServoEnum::RUDDER, "rudder" },
				 { ServoEnum::THROTTLE, "throttle" },
				 { ServoEnum::RPM, "rpm" }
			 }
);

/**
 * @brief SensorData struct. Contains everything needed for calculation of control.
 */
struct SensorData
{
	Vector3 position;                //!< UTM Frame, [X: East, Y: North, Z: Up]
	Vector3 velocity;                //!< Earth Frame
	Vector3 acceleration;            //!< Body Frame
	Vector3 attitude;                //!< [X: Roll, Y: Pitch, Z: Yaw]
	Vector3 angularRate;            //!< [X: Roll, Y: Pitch, Z: Yaw]
	bool hasGPSFix;                    //!< Shows whether the GPS has a fix
	bool autopilotActive;            //!< Shows if the autopilot is active
	FloatingType airSpeed;            //!< total velocity w.r.t. wind
	FloatingType groundSpeed;        //!< total velocity w.r.t. ground
	FloatingType angleOfAttack;        //!< current angle of attack
	FloatingType angleOfSideslip;    //!< current angle of sideslip
	TimePoint timestamp;            //!< Timestamp of SensorEnum data

};

struct PowerData
{
	FloatingType propulsionPower;    //!< measured or estimated current propulsion power
	FloatingType consumedEnergy;    //!< measured or estimated total used energy for propulsion
	FloatingType batteryVoltage;    //!< current battery voltage
	FloatingType batteryCurrent;    //!< current battery current
	TimePoint timestamp;            //!< Timestamp of SensorEnum data
};

struct ServoData
{
	FloatingType aileron;            //!< current aileron position
	FloatingType elevator;            //!< current elevator position
	FloatingType rudder;            //!< current rudder position
	FloatingType throttle;            //!< current throttle position
	FloatingType rpm;                //!< current motor rotation speed
	TimePoint timestamp;            //!< Timestamp of SensorEnum data
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
	ar & t.airSpeed;
	ar & t.groundSpeed;
	ar & t.hasGPSFix;
	ar & t.autopilotActive;
	ar & t.angleOfAttack;
	ar & t.angleOfSideslip;
	ar & t.timestamp;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, PowerData& t)
{
	ar & t.propulsionPower;
	ar & t.consumedEnergy;
	ar & t.batteryVoltage;
	ar & t.batteryCurrent;
	ar & t.timestamp;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, ServoData& t)
{
	ar & t.aileron;
	ar & t.elevator;
	ar & t.rudder;
	ar & t.throttle;
	ar & t.rpm;
	ar & t.timestamp;
}
}

#endif /* UAVAP_CORE_SensorEnumDATA_H_ */
