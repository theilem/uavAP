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
#include "uavAP/Watchdog/ProcessMonitor/ProcessMonitor.h"

ProcessMonitor::ProcessMonitor() :
		binaryPath_("./"), configPath_("./")
{
}

bool
ProcessMonitor::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	Configuration processConfig;

	pm.add("binary_path", binaryPath_, false);
	pm.add("config_path", configPath_, false);
	pm.add("processes", processConfig, true);

	for (auto &it : processConfig)
	{
		ProcessInfo processInfo(it.first);
		if (!processInfo.configure(it.second))
			return false;
		processes_.push_back(std::move(processInfo));
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
		if (it.process.running() && !it.joined)
		{
			kill(it.id, SIGINT);
		}
	}

	if (!tryJoinAll(Seconds(2)))
	{
		CPSLOG_ERROR << "Processes did not join after 2sec and SIGINT. Sending SIGTERM.";
		for (auto& it : processes_)
		{
			if (it.process.running())
			{
				CPSLOG_DEBUG << "Killing " << it.name << " with SIGTERM";
				kill(it.id, SIGTERM);
			}
		}
	}
	else
	{
		CPSLOG_DEBUG << "All processes joined successfully";
	}
}

bool
ProcessMonitor::checkAlive()
{
	bool result = true;
	for (auto& it : processes_)
	{
		if (!it.process.running())
		{
			CPSLOG_ERROR << it.name << " died";
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
			CPSLOG_DEBUG << "Start Process " << it.name;
			CPSLOG_DEBUG << "with absolut path: " << binaryPath_;
			CPSLOG_DEBUG << "with binary path: " << it.binaryPath;
			CPSLOG_DEBUG << "with config path: " << it.configPath;
			it.startChild(binaryPath_, configPath_);
			CPSLOG_DEBUG << "Success. ID: " << it.id;
		} catch (boost::process::process_error& er)
		{
			CPSLOG_ERROR << "Failed to launch process " << it.name << ": " << er.what();
			return false;
		}
	}
	return true;
}

bool
ProcessMonitor::tryJoinAll(Duration timeout)
{
	TimePoint t = Clock::now();

	bool allJoined = true;
	while (Clock::now() - t < timeout)
	{
		allJoined = true;
		for (auto& it : processes_)
		{
			if (!it.joined)
			{
//				if (!it.process.joinable())
//				{
//					allJoined = false;
//					continue;
//				}
				if (it.process.wait_for(Milliseconds(1)))
				{
					CPSLOG_DEBUG << "Joined " << it.name;

					it.joined = true;
				}
				else
					allJoined = false;
			}
		}
		if (allJoined)
			return true;
	}

	return allJoined;
}
