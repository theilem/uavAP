/*
 * LogParser.h
 *
 *  Created on: May 17, 2018
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_LOGPARSER_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_LOGPARSER_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/unordered_map.hpp>
#include <uavAP/API/ap_ext/ap_ext.h>
#include <unordered_map>

#include <cpsCore/Utilities/EnumMap.hpp>
#include <cpsCore/cps_object>
#include <uavAP/API/AutopilotAPI.hpp>
#include "LogParserParams.h"

class IAutopilotAPI;

class IScheduler;

enum class DataID
{
	IMU_PKT,
	INT_IMU_PKT,
	EULER_ROLL,
	EULER_PITCH,
	EULER_YAW,
	QUAT_W,
	QUAT_X,
	QUAT_Y,
	QUAT_Z,
	ACC_X,
	ACC_Y,
	ACC_Z,
	ROLL_RATE,
	PITCH_RATE,
	YAW_RATE,
	LATITUDE,
	LONGITUDE,
	ALTITUDE,
	INT_LATITUDE,
	INT_LONGITUDE,
	INT_ALTITUDE,
	VELOCITY_X,
	VELOCITY_Y,
	VELOCITY_Z,
	COURSE_OVER_GROUND,
	SPEED_OVER_GROUND,
	VERTICAL_VEL,
	AIRSPEED,
	GPS_FIX,
	TIME_DAY,
	TIME_HOUR,
	TIME_MINUTE,
	TIME_MONTH,
	TIME_NANOSEC,
	TIME_SEC,
	TIME_YEAR,
	INT_TIME_DAY,
	INT_TIME_HOUR,
	INT_TIME_MINUTE,
	INT_TIME_MONTH,
	INT_TIME_NANOSEC,
	INT_TIME_SEC,
	INT_TIME_YEAR,
	NUM_OF_IDX
};

ENUMMAP_INIT(DataID, {
	{ DataID::IMU_PKT, "IMU Packet Number" },
	{ DataID::INT_IMU_PKT, "Int. IMU IMU Packet Number" },
	{ DataID::EULER_ROLL, "Euler Angles phi (deg) " },
	{ DataID::EULER_PITCH, "Euler Angles theta (deg) " },
	{ DataID::EULER_YAW, "Euler Angles psi (deg) " },
	{ DataID::QUAT_W, " Quaternion w" },
	{ DataID::QUAT_X, " Quaternion x" },
	{ DataID::QUAT_Y, " Quaternion y" },
	{ DataID::QUAT_Z, " Quaternion z" },
	{ DataID::ACC_X, "Acceleration x (m/s2) " },
	{ DataID::ACC_Y, "Acceleration y (m/s2) " },
	{ DataID::ACC_Z, "Acceleration z (m/s2) " },
	{ DataID::ROLL_RATE, "Rotation Rate x (rad/s) " },
	{ DataID::PITCH_RATE, "Rotation Rate y (rad/s) " },
	{ DataID::YAW_RATE, "Rotation Rate z (rad/s) " },
	{ DataID::LONGITUDE, "Longitude (deg)" },
	{ DataID::LATITUDE, "Latitude (deg)" },
	{ DataID::ALTITUDE, "Altitude (m)" },
	{ DataID::INT_LONGITUDE, "Int. IMU Longitude (deg)" },
	{ DataID::INT_LATITUDE, "Int. IMU Latitude (deg)" },
	{ DataID::INT_ALTITUDE, "Int. IMU Altitude (m)" },
	{ DataID::VELOCITY_X, "Velocity x (m/s) " },
	{ DataID::VELOCITY_Y, "Velocity y (m/s) " },
	{ DataID::VELOCITY_Z, "Velocity z (m/s) " },
	{ DataID::COURSE_OVER_GROUND, "Course over Ground (deg)" },
	{ DataID::SPEED_OVER_GROUND, "Speed over Ground (m/s)" },
	{ DataID::VERTICAL_VEL, "Vertical Velocity (m/s)" },
	{ DataID::TIME_DAY, "UTC Time - Day" },
	{ DataID::TIME_HOUR, "UTC Time - Hour" },
	{ DataID::TIME_MINUTE, "UTC Time - Minute" },
	{ DataID::TIME_MONTH, "UTC Time - Month" },
	{ DataID::TIME_NANOSEC, "UTC Time - Nanoseconds" },
	{ DataID::TIME_SEC, "UTC Time - Second" },
	{ DataID::TIME_YEAR, "UTC Time - Year" },
	{ DataID::INT_TIME_DAY, "Int. IMU UTC Time - Day" },
	{ DataID::INT_TIME_HOUR, "Int. IMU UTC Time - Hour" },
	{ DataID::INT_TIME_MINUTE, "Int. IMU UTC Time - Minute" },
	{ DataID::INT_TIME_MONTH, "Int. IMU UTC Time - Month" },
	{ DataID::INT_TIME_NANOSEC, "Int. IMU UTC Time - Nanoseconds" },
	{ DataID::INT_TIME_SEC, "Int. IMU UTC Time - Second" },
	{ DataID::INT_TIME_YEAR, "Int. IMU UTC Time - Year" },
	{ DataID::GPS_FIX, "GPS Fix" },
	{ DataID::AIRSPEED, "Air Speed (m/s)" }
}
);

class LogParser
		: public AggregatableObject<IScheduler>, public IRunnableObject, public ConfigurableObject<LogParserParams>
{
public:

	static constexpr TypeId typeId = "log_parser";

	LogParser();

	bool
	run(RunStage stage) override;

private:

	bool
	setupLog();

	void
	initializeSample();

	void
	createAndSendSample();

	template<typename T>
	T
	getValue(const std::vector<std::string>& items, DataID id);

	template<typename T>
	T
	getValue(const std::vector<std::string>& items, DataID id, T defaultVal);

	data_sample_t dataSample_;

	std::ifstream logFile_;
	int dataIDIndeces_[static_cast<size_t>(DataID::NUM_OF_IDX)] = {-1};

};

template<typename T>
inline T
LogParser::getValue(const std::vector<std::string>& items, DataID id)
{
	int idx = dataIDIndeces_[static_cast<size_t>(id)];

	if (idx == -1)
	{
		CPSLOG_ERROR << EnumMap<DataID>::convert(id) << " not available" << std::endl;
		throw std::range_error("provided data id not available");
	}

	T ret;
	std::stringstream(items.at(idx)) >> ret;
	return ret;
}

template<typename T>
inline T
LogParser::getValue(const std::vector<std::string>& items, DataID id, T defaultVal)
{
	try
	{
		return getValue<T>(items, id);
	}
	catch (std::range_error& err)
	{
		return defaultVal;
	}
}

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_LOGPARSER_H_ */
