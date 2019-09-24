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
 * SynchronizedRunner.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: uav
 */
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <uavAP/Core/Logging/APLogger.h>
#include "uavAP/Core/Object/Aggregator.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Runner/SynchronizedRunner.h"
#include "uavAP/Core/Runner/SynchronizedRunnerMaster.h"
#include <mutex>
#include <string>

SynchronizedRunner::SynchronizedRunner() :
		sync_(boost::interprocess::open_only, "sync_run", boost::interprocess::read_write)
{
}

bool
SynchronizedRunner::runSynchronized(Aggregator& agg)
{
	using namespace boost::interprocess;
	mapped_region region(sync_, read_write);
	auto synchronizer = static_cast<Synchronizer*>(region.get_address());

	auto runnableObjects = agg.getAll<IRunnableObject>();

	RunStage lastRunStage = RunStage::SYNCHRONIZE;

	while (lastRunStage != RunStage::FINAL)
	{
		RunStage stage;
		std::unique_lock<boost::interprocess::interprocess_mutex> lock(
				synchronizer->runStageMutex);
		if (synchronizer->failure)
			return true;
		stage = synchronizer->runStage;
		if (stage == lastRunStage)
		{
			synchronizer->runStageChanged.wait(lock);
			if (synchronizer->failure)
				return true;
			stage = synchronizer->runStage;
		}
		lock.unlock();

		bool failed = false;
		for (auto it : runnableObjects)
		{
			if (it->run(stage))
				failed = true;
		}
		if (failed)
			return true;
		lastRunStage = stage;

		APLogger::instance()->flush(); //Synchronize stdio
		synchronizer->finishedStage.post();
	}
	return false;
}
