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
 * ProcessInfo.h
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#ifndef UAVAP_WATCHDOG_PROCESSINFO_H_
#define UAVAP_WATCHDOG_PROCESSINFO_H_

#include <string>
#include <cpsCore/Configuration/Configuration.hpp>

// If using macOS do this
#ifdef __APPLE__
#include <boost/process/v1/child.hpp>
using ChildProcess = boost::process::v1::child;
using ProcessError = boost::process::v1::process_error;
#else
#include <boost/process.hpp>
using ChildProcess = boost::process::child;
using ProcessError = boost::process::process_error;
#endif

struct ProcessInfo
{
	std::string name;
	std::string binaryPath;
	std::string configPath;
	ChildProcess process;
	bool joined;
	pid_t id;

	ProcessInfo(const std::string& name);

	bool
	configure(const Configuration& config);

	bool
	startChild(const std::string& binaryPath, const std::string& configPath);

};

#endif /* UAVAP_WATCHDOG_PROCESSINFO_H_ */
