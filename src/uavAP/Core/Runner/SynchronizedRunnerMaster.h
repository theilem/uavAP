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
 * SynchronizedRunnerMaster.h
 *
 *  Created on: Jul 1, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_RUNNER_SYNCHRONIZEDRUNNERMASTER_H_
#define UAVAP_CORE_RUNNER_SYNCHRONIZEDRUNNERMASTER_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_recursive_mutex.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Time.h"

struct Synchronizer
{
	RunStage runStage;
	boost::interprocess::interprocess_condition runStageChanged;
	boost::interprocess::interprocess_mutex runStageMutex;

	boost::interprocess::interprocess_semaphore finishedStage;

	Synchronizer() :
			runStage(RunStage::SYNCHRONIZE), finishedStage(0)
	{
	}
};

class SynchronizedRunnerMaster
{
public:
	SynchronizedRunnerMaster(int numOfRunners);

	~SynchronizedRunnerMaster();

	bool
	runAllStages();

	bool
	runStage(RunStage stage);

private:

	boost::interprocess::shared_memory_object sync_;

	Duration timeout_;

	int numOfRunners_;
};

#endif /* UAVAP_CORE_RUNNER_SYNCHRONIZEDRUNNERMASTER_H_ */
