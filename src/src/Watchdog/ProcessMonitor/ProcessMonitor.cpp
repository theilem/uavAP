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
 * Watchdog.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Watchdog/ProcessMonitor/ProcessMonitor.h"

ProcessMonitor::ProcessMonitor() :
		binaryPath_("./"), configPath_("./")
{
}

bool
ProcessMonitor::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree processConfig;

	pm.add("binary_path", binaryPath_, false);
	pm.add("config_path", configPath_, false);
	pm.add("processes", processConfig, true);

	for (auto &it : processConfig)
	{
		ProcessInfo processInfo(it.first);
		if (!processInfo.configure(it.second))
			return false;
		processes_.insert(std::make_pair(it.first, std::move(processInfo)));
	}
	return pm.map();
}

bool
ProcessMonitor::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		//Setup message_queues
		break;
	}
	case RunStage::NORMAL:
	{
		//Start processes
		break;
	}
	case RunStage::FINAL:
	{
		//Start watchdog monitoring tasks
		break;
	}
	default:
		break;
	}
	return false;
}

void
ProcessMonitor::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

int
ProcessMonitor::getNumOfProcesses()
{
	return processes_.size();
}

void
ProcessMonitor::killAll()
{
	for (auto& it : processes_)
	{
		if (it.second.process.running())
		{
			kill(it.second.id, SIGINT);
			it.second.process.join();
		}
	}
}

bool
ProcessMonitor::checkAlive()
{
	bool result = true;
	for (auto& it : processes_)
	{
		if (!it.second.process.running())
		{
			APLOG_ERROR << it.second.name << " died";
			result = false;
		}
	}
	return result;
}

bool
ProcessMonitor::startAll()
{
	for (auto& it : processes_)
	{
		try
		{
			APLOG_DEBUG << "Start Process " << it.second.name;
			APLOG_DEBUG << "with absolut path: " << binaryPath_;
			APLOG_DEBUG << "with binary path: " << it.second.binaryPath;
			APLOG_DEBUG << "with config path: " << it.second.configPath;
			it.second.startChild(binaryPath_, configPath_);
			APLOG_DEBUG << "Success. ID: " << it.second.id;
		} catch (boost::process::process_error& er)
		{
			APLOG_ERROR << "Failed to launch process " << it.second.name << ": " << er.what();
			return false;
		}
	}
	return true;
}

bool
ProcessMonitor::tryJoinAll(Duration timeout)
{
	TimePoint t = boost::get_system_time();

	bool allJoined = true;
	while (boost::get_system_time() - t < timeout)
	{
		allJoined = true;
		for (auto& it : processes_)
		{
			if (it.second.process.running())
			{
				if (!it.second.process.joinable())
				{
					allJoined = false;
					continue;
				}
				it.second.process.join();
			}
		}
		if (allJoined)
			return true;
	}

	return allJoined;
}
