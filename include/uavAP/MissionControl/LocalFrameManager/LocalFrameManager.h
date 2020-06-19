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

#include "uavAP/Core/Frames/VehicleOneFrame.h"
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

	const VehicleOneFrame&
	getFrame() const;

	void
	setFrame(const VehicleOneFrame& frame);

private:

	void
	publishFrame();

	Publisher<VehicleOneFrame> framePublisher_;
	VehicleOneFrame frame_;
	mutable std::mutex frameMutex_;

	VehicleOneFrame
	localFrameRequest(const DataRequest& request);
};

#endif /* UAVAP_MISSIONCONTROL_LOCALFRAMEMANAGER_LOCALFRAMEMANAGER_H_ */
