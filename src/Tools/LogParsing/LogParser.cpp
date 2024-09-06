/*
 * LogParser.cpp
 *
 *  Created on: May 17, 2018
 *      Author: mircot
 */
#include "LogParser.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <unordered_map>

#include <boost/unordered_map.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/token_functions.hpp>
#include <boost/tokenizer.hpp>

#include <cpsCore/Utilities/Scheduler/IScheduler.h>

LogParser::LogParser()
{
}

bool
LogParser::setupLog()
{
	logFile_.open(params.logFilePath());

	if (!logFile_.is_open())
		return false;

	//Count lines
	std::string str;
	int counter = 0;
	auto offset = logFile_.tellg();
	while (std::getline(logFile_, str))
	{
		counter++;
		if (counter * params.periodUs() < params.offsetSecs() * 1e6)
			offset = logFile_.tellg();
	}
	logFile_.clear();
	logFile_.seekg(offset, std::ios_base::beg);
	if (logFile_.fail())
	{
		CPSLOG_ERROR << "Log file failed";
		return false;
	}
	CPSLOG_DEBUG << "Log length: " << counter * params.periodUs() / 1e6 << " secs";
	CPSLOG_DEBUG << "Starting at sec: " << params.offsetSecs();

	std::ifstream headerFile(params.logHeaderPath());
	std::string header;
	std::getline(headerFile, header);

	boost::char_separator<char> sep
			{";"};
	boost::tokenizer<boost::char_separator<char>> token
			{header, sep};

	std::unordered_map<std::string, int> headerIdcs;
	int k = 0;
	for (auto& tok : token)
	{
		headerIdcs.insert(std::make_pair(tok, k));
		k++;
	}

	CPSLOG_TRACE << "Map log header to data indeces";
	for (size_t i = 0; i < static_cast<size_t>(DataID::NUM_OF_IDX); ++i)
	{
		auto it = EnumMap<DataID>::convert(static_cast<DataID>(i));

		auto index = headerIdcs.find(it);
		if (index != headerIdcs.end())
		{
			dataIDIndeces_[i] = index->second;
			CPSLOG_TRACE << it << ": " << dataIDIndeces_[i];
		}
	}
	return true;
}

void
LogParser::initializeSample()
{
	if (params.internalImu())
		dataSample_.int_imu_sample = new imu_sample_t;
	else
		dataSample_.imu_sample = new imu_sample_t;

	if (params.externalGps())
		dataSample_.pic_sample = new pic_sample_t;

	if (params.useAirspeed())
		dataSample_.airs_sample = new airs_sample_t;
}

bool
LogParser::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<IScheduler>())
			{
				CPSLOG_ERROR << "Log parser: missing deps.";
				return true;
			}

			if (!setupLog())
			{
				CPSLOG_ERROR << "Setting up log file failed";
				return true;
			}
			initializeSample();


			setConfigPath(params.alvoloConfigPath());

			if (ap_ext_setup() != 0)
				return true;
			break;
		}
		case RunStage::NORMAL:
		{
			auto scheduler = get<IScheduler>();
			scheduler->schedule([this] { createAndSendSample(); }, Milliseconds(0),
								Microseconds(params.periodUs()));
			break;
		}
		default:
			break;
	}
	return false;

}

void
LogParser::createAndSendSample()
{
	if (!logFile_.is_open())
	{
		CPSLOG_ERROR << "Log file not open. Cannot populate data sample.";
		return;
	}

	std::string line;
	if (!std::getline(logFile_, line))
	{
		if (logFile_.eof())
			CPSLOG_ERROR << "Logfile end of file";
		else
			CPSLOG_ERROR << "Unknown error";
		return;
	}
	std::vector<std::string> items;

	boost::split(items, line, boost::is_any_of(";"), boost::token_compress_off);

	if (params.internalImu())
	{
		auto intimu = dataSample_.int_imu_sample;

		if (intimu)
		{
			intimu->imu_pkt = getValue<unsigned long>(items, DataID::INT_IMU_PKT);

			if (params.useEuler())
			{
				intimu->imu_euler_roll = getValue<double>(items, DataID::EULER_ROLL);
				intimu->imu_euler_pitch = getValue<double>(items, DataID::EULER_PITCH);
				intimu->imu_euler_yaw = getValue<double>(items, DataID::EULER_YAW);
			}
			else
			{
				intimu->imu_quat_w = getValue<double>(items, DataID::QUAT_W);
				intimu->imu_quat_x = getValue<double>(items, DataID::QUAT_X);
				intimu->imu_quat_y = getValue<double>(items, DataID::QUAT_Y);
				intimu->imu_quat_z = getValue<double>(items, DataID::QUAT_Z);
			}

			intimu->imu_accel_x = getValue<double>(items, DataID::ACC_X);
			intimu->imu_accel_y = getValue<double>(items, DataID::ACC_Y);
			intimu->imu_accel_z = getValue<double>(items, DataID::ACC_Z);

			intimu->imu_rot_x = getValue<double>(items, DataID::ROLL_RATE);
			intimu->imu_rot_y = getValue<double>(items, DataID::PITCH_RATE);
			intimu->imu_rot_z = getValue<double>(items, DataID::YAW_RATE);

			if (!params.externalGps())
			{
				intimu->imu_lon = getValue<double>(items, DataID::INT_LONGITUDE);
				intimu->imu_lat = getValue<double>(items, DataID::INT_LATITUDE);
				intimu->imu_alt = getValue<double>(items, DataID::INT_ALTITUDE);

				// Velocity
				intimu->imu_vel_x = getValue<double>(items, DataID::VELOCITY_X);
				intimu->imu_vel_y = getValue<double>(items, DataID::VELOCITY_Y);
				intimu->imu_vel_z = getValue<double>(items, DataID::VELOCITY_Z);

				//Valid flag for GPS fix
				intimu->valid_flags = static_cast<unsigned long>(0x80);

				intimu->imu_time_year = getValue<double>(items, DataID::INT_TIME_YEAR);
				intimu->imu_time_month = getValue<double>(items, DataID::INT_TIME_MONTH);
				intimu->imu_time_day = getValue<double>(items, DataID::INT_TIME_DAY);
				intimu->imu_time_hour = getValue<double>(items, DataID::INT_TIME_HOUR);
				intimu->imu_time_minute = getValue<double>(items, DataID::INT_TIME_MINUTE);
				intimu->imu_time_second = getValue<double>(items, DataID::INT_TIME_SEC);
				intimu->imu_time_nano = getValue<double>(items, DataID::INT_TIME_NANOSEC);
			}
		}
	}
	else
	{
		auto imu = dataSample_.imu_sample;

		if (imu)
		{
			imu->imu_pkt = getValue<unsigned long>(items, DataID::IMU_PKT);

			if (params.useEuler())
			{
				imu->imu_euler_roll = getValue<double>(items, DataID::EULER_ROLL);
				imu->imu_euler_pitch = getValue<double>(items, DataID::EULER_PITCH);
				imu->imu_euler_yaw = getValue<double>(items, DataID::EULER_YAW);
			}
			else
			{
				imu->imu_quat_w = getValue<double>(items, DataID::QUAT_W);
				imu->imu_quat_x = getValue<double>(items, DataID::QUAT_X);
				imu->imu_quat_y = getValue<double>(items, DataID::QUAT_Y);
				imu->imu_quat_z = getValue<double>(items, DataID::QUAT_Z);
			}

			imu->imu_accel_x = getValue<double>(items, DataID::ACC_X);
			imu->imu_accel_y = getValue<double>(items, DataID::ACC_Y);
			imu->imu_accel_z = getValue<double>(items, DataID::ACC_Z);

			imu->imu_rot_x = getValue<double>(items, DataID::ROLL_RATE);
			imu->imu_rot_y = getValue<double>(items, DataID::PITCH_RATE);
			imu->imu_rot_z = getValue<double>(items, DataID::YAW_RATE);

			if (!params.externalGps())
			{
				imu->imu_lon = getValue<double>(items, DataID::LONGITUDE);
				imu->imu_lat = getValue<double>(items, DataID::LATITUDE);
				imu->imu_alt = getValue<double>(items, DataID::ALTITUDE);

				// Velocity
				imu->imu_vel_x = getValue<double>(items, DataID::VELOCITY_X);
				imu->imu_vel_y = getValue<double>(items, DataID::VELOCITY_Y);
				imu->imu_vel_z = getValue<double>(items, DataID::VELOCITY_Z);

				//Valid flag for GPS fix
				imu->valid_flags = static_cast<unsigned long>(0x80);

				imu->imu_time_year = getValue<double>(items, DataID::TIME_YEAR);
				imu->imu_time_month = getValue<double>(items, DataID::TIME_MONTH);
				imu->imu_time_day = getValue<double>(items, DataID::TIME_DAY);
				imu->imu_time_hour = getValue<double>(items, DataID::TIME_HOUR);
				imu->imu_time_minute = getValue<double>(items, DataID::TIME_MINUTE);
				imu->imu_time_second = getValue<double>(items, DataID::TIME_SEC);
				imu->imu_time_nano = getValue<double>(items, DataID::TIME_NANOSEC);
			}
		}

		if (params.externalGps())
		{
			auto pic = dataSample_.pic_sample;

			if (pic)
			{
				//only update if gps is valid:
				if (getValue<int>(items, DataID::GPS_FIX) == 7)
				{
					auto& pos = pic->gps_sample.position;

					pos.flags = 7;
					pos.longitude = getValue<double>(items, DataID::LONGITUDE);
					pos.latitude = getValue<double>(items, DataID::LATITUDE);
					pos.msl_altitude = getValue<double>(items, DataID::ALTITUDE);

					pos.course_gnd = getValue<double>(items, DataID::COURSE_OVER_GROUND);
					pos.speed_gnd_kh = getValue<double>(items, DataID::SPEED_OVER_GROUND);
					pos.vert_velocity = getValue<double>(items, DataID::VERTICAL_VEL);
				}
			}
		}
	}

	if (params.useAirspeed())
	{
		auto airs = dataSample_.airs_sample;
		if (airs)
		{
			airs->cal_airs = getValue<double>(items, DataID::AIRSPEED);
		}
	}

	int sensResult = ap_ext_sense(&dataSample_);

	if (sensResult != 0)
		CPSLOG_ERROR << "ap_ext_sense returned with " << sensResult;

}
