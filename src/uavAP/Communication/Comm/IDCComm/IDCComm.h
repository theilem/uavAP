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
/**
 *  @file         SerialComm.h
 *  @author Simon Yu
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Serial Comm Header File
 *
 *  Description
 */

#ifndef UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_
#define UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_

#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/IPC/Subscription.h>


#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Communication/Comm/IComm.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IDC/IDCSender.h"
#include <mutex>

class IScheduler;
class DataPresentation;
class IDC;
class IPC;

class IDCComm: public IComm, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "idc";

	IDCComm();

	static std::shared_ptr<IDCComm>
	create(const Configuration& configuration);

	bool
	configure(const Configuration& configuration);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	void
	sendPacket(const Packet& packet);

	void
	receivePacket(const Packet& packet);

	void
	tryConnectChannelMixing();

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IDC> idc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<DataPresentation> dataPresentation_;

	Subscription flightAnalysisSubscription_;
	Subscription flightControlSubscription_;
	Subscription missionControlSubscription_;
	Subscription channelMixingSubscription_;
	Publisher<Packet> flightAnalysisPublisher_;
	Publisher<Packet> flightControlPublisher_;
	Publisher<Packet> missionControlPublisher_;
	Publisher<Packet> apiPublisher_;

	std::mutex senderMutex_;
	IDCSender sender_;
	boost::signals2::connection idcConnection_;
};

#endif /* UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_MM_H_ */
