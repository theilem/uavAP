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
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h>
#include <cpsCore/Utilities/IPC/IPC.h>

LocalFrameManager::LocalFrameManager() :
		frame_(0)
{
}

bool
LocalFrameManager::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC,IScheduler>())
		{
			CPSLOG_ERROR << "LocalFrameManger: missing deps";
			return true;
		}
		frame_ = VehicleOneFrame(params.yaw()(), params.origin());

		auto ipc = get<IPC>();


		framePublisher_ = ipc->publish<VehicleOneFrame>("local_frame");
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = get<IScheduler>();
		scheduler->schedule(std::bind(&LocalFrameManager::publishFrame, this), Milliseconds(0), Milliseconds(500));
		break;
	}
	default:
		break;
	}
	return false;
}

void
LocalFrameManager::publishFrame()
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	framePublisher_.publish(frame_);
}

const VehicleOneFrame&
LocalFrameManager::getFrame() const
{
	std::lock_guard<std::mutex> lg(frameMutex_);
	return frame_;
}

void
LocalFrameManager::setFrame(VehicleOneFrame frame)
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	frame_ = frame;
	lock.unlock();
}
