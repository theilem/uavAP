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
 * ProcessInfo.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#include <cpsCore/Configuration/Configuration.hpp>
#include <cpsCore/Configuration/PropertyMapper.hpp>
#include <cpsCore/Logging/CPSLogger.h>
#include "uavAP/Watchdog/ProcessMonitor/ProcessInfo.h"

ProcessInfo::ProcessInfo(const std::string& n) :
		name(n), joined(false), id(0)
{
}

bool
ProcessInfo::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	pm.add("binary", binaryPath, true);
	pm.add("config", configPath, false);
	return pm.map();
}

bool
ProcessInfo::startChild(const std::string& binPath, const std::string& confPath)
{
	if (binaryPath.empty())
	{
		CPSLOG_ERROR << "Binary path missing";
		return false;
	}
	if (configPath.empty())
	{
		CPSLOG_DEBUG << "No config path. Starting " << binPath + binaryPath;
		process = boost::process::child(binPath + binaryPath);
	}
	else
	{
		CPSLOG_DEBUG << "Starting " << binPath + binaryPath << " with " << confPath + configPath;
		process = boost::process::child(binPath + binaryPath, confPath + configPath);
	}
	id = process.id();
	return true;
}
