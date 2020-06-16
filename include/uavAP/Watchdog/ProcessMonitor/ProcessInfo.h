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

#include <boost/process.hpp>
#include <string>
#include <cpsCore/Utilities/Time.hpp>
#include <cpsCore/Configuration/Configuration.hpp>

struct ProcessInfo
{
	std::string name;
	std::string binaryPath;
	std::string configPath;
	boost::process::child process;
	bool joined;
	pid_t id;

	ProcessInfo(const std::string& name);

	bool
	configure(const Configuration& config);

	bool
	startChild(const std::string& binaryPath, const std::string& configPath);

};

#endif /* UAVAP_WATCHDOG_PROCESSINFO_H_ */
