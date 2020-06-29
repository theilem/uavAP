/*
 * LocalFrameManager.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: mircot
 */
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include <uavAP/Core/DataHandling/DataHandling.h>

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

		if (!isSet<DataHandling>())
		{
			CPSLOG_WARN << "LocalFrameManager: Cannot send local frame. DataHandling missing";
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

		if (auto dh = get<DataHandling>())
		{
			dh->addTriggeredStatusFunction<VehicleOneFrame, DataRequest>(
					std::bind(&LocalFrameManager::localFrameRequest, this,
							  std::placeholders::_1), Content::LOCAL_FRAME, Content::REQUEST_DATA);
		}
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
LocalFrameManager::setFrame(const VehicleOneFrame& frame)
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	frame_ = frame;
	lock.unlock();
}



Optional<VehicleOneFrame>
LocalFrameManager::localFrameRequest(const DataRequest& request)
{
	if (request == DataRequest::LOCAL_FRAME)
		return frame_;
	return std::nullopt;
}

