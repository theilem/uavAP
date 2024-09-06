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
#include <cpsCore/Utilities/NamedValue.hpp>
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
	COURSE,
	TEMPERATURE,
	PRESSURE,
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
				 { SensorEnum::ANGLE_OF_SIDESLIP, "angle_of_sideslip" },
				 { SensorEnum::TEMPERATURE, "temperature" },
				 { SensorEnum::COURSE, "course" },
				 { SensorEnum::PRESSURE, "pressure" },
				 { SensorEnum::GROUND_SPEED, "ground_speed" }
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
	Vector3 position = {0., 0., 0.};                //!< UTM Frame, [X: East, Y: North, Z: Up]
	Vector3 velocity = {0., 0., 0.};                //!< Earth Frame
	Vector3 acceleration = {0., 0., 0.};            //!< Body Frame
	Vector3 attitude = {0., 0., 0.};                //!< [X: Roll, Y: Pitch, Z: Yaw]
	Vector3 angularRate = {0., 0., 0.};            //!< [X: Roll, Y: Pitch, Z: Yaw]
	bool hasGPSFix = false;                    //!< Shows whether the GPS has a fix
	bool autopilotActive = false;            //!< Shows if the autopilot is active
	FloatingType airSpeed = 0.;            //!< total velocity w.r.t. wind
	FloatingType groundSpeed = 0.;        //!< total velocity w.r.t. ground
	FloatingType angleOfAttack = 0.;        //!< current angle of attack
	FloatingType angleOfSideslip = 0.;    //!< current angle of sideslip
	FloatingType courseAngle = 0.;       //!< course angle obtained from GPS
	FloatingType pressure = 0.;       //!< pressure obtained from airspeed sensor
	FloatingType temperature = 0.;     //!< temperature obtained from airspeed sensor
	Frame frame = Frame::INERTIAL;
	TimePoint timestamp;            //!< Timestamp of SensorEnum data
	uint32_t sequenceNumber;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct NamedSensorData
{
	static constexpr auto name = "sensor_data";
	NamedValue<Vector3> position = {Vector3(0., 0., 0.), "position"};
	NamedValue<Vector3> velocity = {Vector3(0., 0., 0.), "velocity"};
	NamedValue<Vector3> acceleration = {Vector3(0., 0., 0.), "acceleration"};
	NamedValue<Vector3> attitude = {Vector3(0., 0., 0.), "attitude"};
	NamedValue<Vector3> angularRate = {Vector3(0., 0., 0.), "angularRate"};
	NamedValue<bool> hasGPSFix = {false, "hasGPSFix"};
	NamedValue<bool> autopilotActive = {false, "autopilotActive"};
	NamedValue<FloatingType> airSpeed = {0., "airSpeed"};
	NamedValue<FloatingType> groundSpeed = {0., "groundSpeed"};
	NamedValue<FloatingType> angleOfAttack = {0., "angleOfAttack"};
	NamedValue<FloatingType> angleOfSideslip = {0., "angleOfSideslip"};
	NamedValue<FloatingType> courseAngle = {0., "courseAngle"};
	NamedValue<FloatingType> pressure = {0., "pressure"};
	NamedValue<FloatingType> temperature = {0., "temperature"};
	NamedValue<Frame> frame = {Frame::INERTIAL, "frame"};
	NamedValue<TimePoint> timestamp = {TimePoint(), "timestamp"};
	NamedValue<uint32_t> sequenceNumber = {0, "sequenceNumber"};

	NamedSensorData() = default;

	explicit NamedSensorData(const SensorData& sd)
	{
		position = sd.position;
		velocity = sd.velocity;
		acceleration = sd.acceleration;
		attitude = sd.attitude;
		angularRate = sd.angularRate;
		hasGPSFix = sd.hasGPSFix;
		autopilotActive = sd.autopilotActive;
		airSpeed = sd.airSpeed;
		groundSpeed = sd.groundSpeed;
		angleOfAttack = sd.angleOfAttack;
		angleOfSideslip = sd.angleOfSideslip;
		courseAngle = sd.courseAngle;
		pressure = sd.pressure;
		temperature = sd.temperature;
		frame = sd.frame;
		timestamp = sd.timestamp;
		sequenceNumber = sd.sequenceNumber;
	}

	template <typename Parser>
	void
	parse(Parser& p) const
	{
		p & position;
		p & velocity;
		p & acceleration;
		p & attitude;
		p & angularRate;
		p & hasGPSFix;
		p & autopilotActive;
		p & airSpeed;
		p & groundSpeed;
		p & angleOfAttack;
		p & angleOfSideslip;
		p & courseAngle;
		p & pressure;
		p & temperature;
		p & frame;
		p & timestamp;
		p & sequenceNumber;
	}

};

template<typename RetType>
RetType
enumAccess(const SensorData& data, const SensorEnum& e)
{
	switch (e)
	{
		case SensorEnum::POSITION_X:
			return static_cast<RetType>(data.position.x());
		case SensorEnum::POSITION_Y:
			return static_cast<RetType>(data.position.y());
		case SensorEnum::POSITION_Z:
			return static_cast<RetType>(data.position.z());
		case SensorEnum::VELOCITY_X:
			return static_cast<RetType>(data.velocity.x());
		case SensorEnum::VELOCITY_Y:
			return static_cast<RetType>(data.velocity.y());
		case SensorEnum::VELOCITY_Z:
			return static_cast<RetType>(data.velocity.z());
		case SensorEnum::ACCELERATION_X:
			return static_cast<RetType>(data.acceleration.x());
		case SensorEnum::ACCELERATION_Y:
			return static_cast<RetType>(data.acceleration.y());
		case SensorEnum::ACCELERATION_Z:
			return static_cast<RetType>(data.acceleration.z());
		case SensorEnum::ATTITUDE_X:
			return static_cast<RetType>(data.attitude.x());
		case SensorEnum::ATTITUDE_Y:
			return static_cast<RetType>(data.attitude.y());
		case SensorEnum::ATTITUDE_Z:
			return static_cast<RetType>(data.attitude.z());
		case SensorEnum::ANGULAR_RATE_X:
			return static_cast<RetType>(data.angularRate.x());
		case SensorEnum::ANGULAR_RATE_Y:
			return static_cast<RetType>(data.angularRate.y());
		case SensorEnum::ANGULAR_RATE_Z:
			return static_cast<RetType>(data.angularRate.z());
		case SensorEnum::AIR_SPEED:
			return static_cast<RetType>(data.airSpeed);
		case SensorEnum::GROUND_SPEED:
			return static_cast<RetType>(data.groundSpeed);
		case SensorEnum::ANGLE_OF_ATTACK:
			return static_cast<RetType>(data.angleOfAttack);
		case SensorEnum::ANGLE_OF_SIDESLIP:
			return static_cast<RetType>(data.angleOfSideslip);
		case SensorEnum::COURSE:
			return static_cast<RetType>(data.courseAngle);
		case SensorEnum::TEMPERATURE:
			return static_cast<RetType>(data.temperature);
		case SensorEnum::PRESSURE:
			return static_cast<RetType>(data.pressure);
		case SensorEnum::SEQUENCE_NR:
			return static_cast<RetType>(data.sequenceNumber);
		case SensorEnum::HAS_GPS_FIX:
			return static_cast<RetType>(data.hasGPSFix);
		default:
			return enumAccessUnknown<RetType>(e);
	}
}

struct PowerData
{
	FloatingType propulsionPower = 0.;    //!< measured or estimated current propulsion power
	FloatingType consumedEnergy = 0.;    //!< measured or estimated total used energy for propulsion
	FloatingType batteryVoltage = 0.;    //!< current battery voltage
	FloatingType batteryCurrent = 0.;    //!< current battery current
	TimePoint timestamp;            //!< Timestamp of PowerData data
};

template<typename RetType>
RetType
enumAccess(const PowerData& data, const PowerEnum& e)
{
	switch (e)
	{
		case PowerEnum::PROPULSION_POWER:
			return static_cast<RetType>(data.propulsionPower);
		case PowerEnum::CONSUMED_ENERGY:
			return static_cast<RetType>(data.consumedEnergy);
		case PowerEnum::BATTERY_VOLTAGE:
			return static_cast<RetType>(data.batteryVoltage);
		case PowerEnum::BATTERY_CURRENT:
			return static_cast<RetType>(data.batteryCurrent);
		default:
			return enumAccessUnknown<RetType>(e);
	}
}

struct ServoData
{
	FloatingType aileron = 0.;            //!< current aileron position
	FloatingType elevator = 0.;            //!< current elevator position
	FloatingType rudder = 0.;            //!< current rudder position
	FloatingType throttle = 0.;            //!< current throttle position
	FloatingType rpm = 0.;                //!< current motor rotation speed
	TimePoint timestamp;            //!< Timestamp of ServoData data
};

template<typename RetType>
RetType
enumAccess(const ServoData& data, const ServoEnum& e)
{
	switch (e)
	{
		case ServoEnum::AILERON:
			return static_cast<RetType>(data.aileron);
		case ServoEnum::ELEVATOR:
			return static_cast<RetType>(data.elevator);
		case ServoEnum::RUDDER:
			return static_cast<RetType>(data.rudder);
		case ServoEnum::THROTTLE:
			return static_cast<RetType>(data.throttle);
		case ServoEnum::RPM:
			return static_cast<RetType>(data.rpm);
		default:
			return enumAccessUnknown<RetType>(e);
	}
}

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
	ar & t.courseAngle;
	ar & t.temperature;
	ar & t.pressure;
	ar & t.frame;
	ar & t.timestamp;
	ar & t.sequenceNumber;
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
