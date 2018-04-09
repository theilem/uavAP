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
 * ApExtManager.h
 *
 *  Created on: Aug 31, 2017
 *      Author: mircot
 */

#ifndef UAVAP_API_APEXTMANAGER_H_
#define UAVAP_API_APEXTMANAGER_H_
#include "uavAP/API/ap_ext/ap_ext.h"
#include "uavAP/API/ap_ext/ServoMapping.h"
#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/Aggregator.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <mutex>
#include <functional>

class IPC;
class SystemTimeProvider;
class MultiThreadingScheduler;

template <typename C, typename T>
class APDataPresentation;

class ChannelMixing;

class ApExtManager
{
public:

	ApExtManager();

	bool
	configure(const boost::property_tree::ptree& config);

	int
	ap_sense(const data_sample_t * sample);

	int
	ap_actuate(unsigned long * pwm, unsigned int num_channels);

	struct OutPWM
	{
		unsigned long ch[7];
	};

	using NotifyActuation = std::function<void()>;

	void
	notifyOnActuation(const NotifyActuation& slot);

private:

	void
	onControllerOutput(const ControllerOutput& output);

	void
	onServoOutput(const OutPWM& output);

	void
	tryConnectControllerOut();

	std::shared_ptr<IPC> ipc_;
	std::shared_ptr<SystemTimeProvider> timeProvider_;
	std::shared_ptr<MultiThreadingScheduler> scheduler_;
	std::shared_ptr<APDataPresentation<Content, Target>> dataPresentation_;
	std::shared_ptr<ChannelMixing> channelMixing_;
	Aggregator agg_;
	ServoMapping servoMapping_;

	std::mutex outputMutex_;
	std::vector<unsigned long> outputChannels_;

	Publisher sensorDataPublisher_;
	Publisher outputPublisher_;
	Subscription controllerOutSubscription_;
	Subscription servoOutSubscription_;

	NotifyActuation notifyActuation_;

};




#endif /* UAVAP_API_APEXTMANAGER_H_ */
