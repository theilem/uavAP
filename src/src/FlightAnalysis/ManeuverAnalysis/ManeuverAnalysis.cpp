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
/*
 * ManeuverAnalysis.cpp
 *
 *  Created on: Aug 6, 2018
 *      Author: simonyu
 */

#include <string>

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightAnalysis/ManeuverAnalysis/ManeuverAnalysis.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

ManeuverAnalysis::ManeuverAnalysis() :
		collectInit_(false), counter_(0)
{
}

std::shared_ptr<ManeuverAnalysis>
ManeuverAnalysis::create(const boost::property_tree::ptree& config)
{
	auto maneuverAnalysis = std::make_shared<ManeuverAnalysis>();

	if (!maneuverAnalysis->configure(config))
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Load Config.";
	}

	return maneuverAnalysis;
}

bool
ManeuverAnalysis::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add("log_path", logPath_, true);

	collectInit_ = false;
	counter_ = 0;

	return pm.map();
}

bool
ManeuverAnalysis::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipcHandle_.isSet())
		{
			APLOG_ERROR << "IPC Missing.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipcHandle_.get();

		sensorDataSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
				std::bind(&ManeuverAnalysis::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			APLOG_ERROR << "Sensor Data Subscription Missing.";
			return true;
		}

		maneuverAnalysisSubscription_ = ipc->subscribeOnPacket("maneuver_analysis_status",
				std::bind(&ManeuverAnalysis::onManeuverAnalysisStatus, this,
						std::placeholders::_1));

		if (!maneuverAnalysisSubscription_.connected())
		{
			APLOG_ERROR << "Maneuver Analysis Subscription Missing.";
			return true;
		}

		break;
	}
	default:
	{
		break;
	}
	}

	return false;
}

void
ManeuverAnalysis::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipcHandle_.setFromAggregationIfNotSet(agg);
}

void
ManeuverAnalysis::onSensorData(const SensorData& data)
{
	std::unique_lock<std::mutex> lock(maneuverAnalysisStatusMutex_);
	ManeuverAnalysisStatus analysis = analysis_;
	lock.unlock();

	if (analysis.analysis && !collectInit_)
	{
		collectStateInit(analysis.maneuver, analysis_.interrupted, data);
	}
	else if (analysis.analysis && collectInit_)
	{
		collectStateNormal(data);
	}
	else if (!analysis.analysis && collectInit_)
	{
		collectStateFinal();
	}
}

void
ManeuverAnalysis::onManeuverAnalysisStatus(const Packet& status)
{
	std::unique_lock<std::mutex> lock(maneuverAnalysisStatusMutex_);
	analysis_ = dp::deserialize<ManeuverAnalysisStatus>(status);
	lock.unlock();
}

void
ManeuverAnalysis::collectStateInit(const std::string& maneuver, const bool& interrupted,
		const SensorData& data)
{
	APLOG_DEBUG << "ManeuverAnalysis::collectStateInit.";

	if (!interrupted)
	{
		counter_ ++;
	}

	std::string maneuverName = std::to_string(counter_) + "_" + maneuver;
	std::string logFileName = logPath_ + maneuverName + ".log";
	std::string time = to_simple_string(data.timestamp);

	logFile_.open(logFileName);
	logFile_.precision(15);

	if (!logFile_.is_open())
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Open Log File. Cannot Log.";
		return;
	}

	logFile_ << maneuverName << std::endl;
	logFile_ << time << std::endl;
	logFile_ << "collectStateInit:" << std::endl;
	logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
			<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
			<< std::endl;
	logFile_ << "collectStateNormal:" << std::endl;

	collectInit_ = true;
}

void
ManeuverAnalysis::collectStateNormal(const SensorData& data)
{
	APLOG_DEBUG << "ManeuverAnalysis::collectStateNormal.";

	if (!logFile_.is_open())
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Open Log File. Cannot Log.";
		return;
	}

	logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
			<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
			<< std::endl;
}

void
ManeuverAnalysis::collectStateFinal()
{
	APLOG_DEBUG << "ManeuverAnalysis::collectStateFinal.";

	if (!logFile_.is_open())
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Open Log File. Cannot Log.";
		return;
	}

	logFile_ << "collectStateFinal:" << std::endl;

	velocity_.clear();
	rollRate_.clear();
	logFile_.close();

	collectInit_ = false;
}
