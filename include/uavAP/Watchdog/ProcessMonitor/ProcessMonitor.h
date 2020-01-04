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
 * Watchdog.h
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#ifndef UAVAP_WATCHDOG_PROCESSMONITOR_H_
#define UAVAP_WATCHDOG_PROCESSMONITOR_H_

#include <cpsCore/cps_object>
#include "uavAP/Watchdog/ProcessMonitor/ProcessInfo.h"
#include <cstdlib>

class ProcessMonitor: public IRunnableObject, public AggregatableObject<>
{
public:

	static constexpr TypeId typeId = "process_monitor";

	ProcessMonitor();

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

	int
	getNumOfProcesses();

	void
	killAll();

	bool
	checkAlive();

	bool
	startAll();

private:

	bool
	tryJoinAll(Duration timeout);

	std::vector<ProcessInfo> processes_;

	std::string binaryPath_;
	std::string configPath_;
};

#endif /* UAVAP_WATCHDOG_PROCESSMONITOR_H_ */
