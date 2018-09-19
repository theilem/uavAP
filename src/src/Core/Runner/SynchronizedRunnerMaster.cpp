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
 * SynchronizedRunnerMaster.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: uav
 */

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/thread/lock_types.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Runner/SynchronizedRunnerMaster.h"

SynchronizedRunnerMaster::SynchronizedRunnerMaster(int numOfRunners) :
		timeout_(Seconds(1)), numOfRunners_(numOfRunners)
{
	using namespace boost::interprocess;
	try
	{
		sync_ = shared_memory_object(create_only, "sync_run", read_write);
	} catch (interprocess_exception&)
	{
		APLOG_WARN << "Synchronizer already exists. Trying to use existing. Might fail.";
		sync_ = shared_memory_object(open_only, "sync_run", read_write);
	}
	sync_.truncate(sizeof(Synchronizer));
	mapped_region region(sync_, read_write);

	void* address = region.get_address();
	Synchronizer init;
	memcpy(address, &init, sizeof(Synchronizer));
}

SynchronizedRunnerMaster::~SynchronizedRunnerMaster()
{
	sync_.remove(sync_.get_name());
}

bool
SynchronizedRunnerMaster::runAllStages()
{
	APLOG_DEBUG << "Run Stage INIT";
	if (runStage(RunStage::INIT))
		return true;
	APLOG_DEBUG << "Run Stage NORMAL";
	if (runStage(RunStage::NORMAL))
		return true;
	APLOG_DEBUG << "Run Stage FINAL";
	if (runStage(RunStage::FINAL))
		return true;
	APLOG_DEBUG << "All stages succeeded";
	return false;
}

bool
SynchronizedRunnerMaster::runStage(RunStage stage)
{
	using namespace boost::interprocess;
	mapped_region region(sync_, read_write);
	auto synchronizer = static_cast<Synchronizer*>(region.get_address());

	boost::unique_lock<boost::interprocess::interprocess_mutex> lock(synchronizer->runStageMutex);
	synchronizer->runStage = stage;
	synchronizer->runStageChanged.notify_all();
	lock.unlock();

	TimePoint timeout = boost::get_system_time() + timeout_;

	for (int i = 0; i < numOfRunners_; ++i)
	{
		if (!synchronizer->finishedStage.timed_wait(timeout))
		{
			APLOG_ERROR << numOfRunners_ - i << " Module(s) timed out at runstage " << (int) stage;
			return true;
		}
	}
	return false;
}
