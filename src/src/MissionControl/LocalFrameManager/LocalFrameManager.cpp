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
 * LocalFrameManager.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: mircot
 */
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h>

LocalFrameManager::LocalFrameManager() :
		frame_(0)
{
}

bool
LocalFrameManager::configure(const Configuration& config)
{
	PropertyMapper pm(config);
	double yaw = 0;
	Vector3 origin(0,0,0);
	pm.add<double>("yaw", yaw, true);
	pm.add("origin", origin, true);

	frame_ = VehicleOneFrame(yaw * M_PI/180.0, origin);
	return pm.map();
}

void
LocalFrameManager::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
}

bool
LocalFrameManager::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "LocalFrameManger: IPC missing";
			return true;
		}

		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "LocalFrameManger: Scheduler missing";
			return true;
		}

		auto ipc = ipc_.get();

		framePublisher_ = ipc->publishOnSharedMemory<VehicleOneFrame>("local_frame");
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&LocalFrameManager::publishFrame, this), Milliseconds(0), Milliseconds(500));
		break;
	}
	default:
		break;
	}
	return false;
}

const VehicleOneFrame&
LocalFrameManager::getFrame() const
{
	return frame_;
}

void
LocalFrameManager::publishFrame()
{
	framePublisher_.publish(frame_);
}
