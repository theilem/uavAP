/*
 * LocalFrameManager.h
 *
 *  Created on: Aug 9, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_LOCALFRAMEMANAGER_LOCALFRAMEMANAGER_H_
#define UAVAP_MISSIONCONTROL_LOCALFRAMEMANAGER_LOCALFRAMEMANAGER_H_

#include <mutex>

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/IPC/Publisher.h>
#include <uavAP/Core/DataHandling/Content.hpp>

#include "uavAP/Core/Frames/LocalFrame.h"
#include "uavAP/MissionControl/LocalFrameManager/LocalFrameManagerParams.h"

class IPC;

class DataHandling;

class IScheduler;

class LocalFrameManager
		: public AggregatableObject<IPC, IScheduler, DataHandling>,
		  public ConfigurableObject<LocalFrameManagerParams>,
		  public IRunnableObject
{

public:

	static constexpr TypeId typeId = "local_frame";

	LocalFrameManager();

	bool
	run(RunStage stage) override;

private:

	void
	publishFrame();

	Optional<LocalFrame>
	localFrameRequest(const DataRequest& request);

	Publisher<LocalFrame> framePublisher_;
	mutable std::mutex frameMutex_;

};

#endif /* UAVAP_MISSIONCONTROL_LOCALFRAMEMANAGER_LOCALFRAMEMANAGER_H_ */
