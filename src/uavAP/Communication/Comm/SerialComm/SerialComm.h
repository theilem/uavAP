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

#ifndef UAVAP_COMMUNICATION_COMM_SERIALCOMM_SERIALCOMM_H_
#define UAVAP_COMMUNICATION_COMM_SERIALCOMM_SERIALCOMM_H_

#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/IDC/Serial/SerialIDC.h"

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Communication/Comm/IComm.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IPC/IPC.h"

class IScheduler;
enum class Content;
enum class Target;

template<typename C, typename T>
class IDataPresentation;

class SerialComm : public IComm, public IAggregatableObject, public IRunnableObject
{
public:
	SerialComm();

	static std::shared_ptr<SerialComm>
	create(const boost::property_tree::ptree& configuration);

	bool
	configure(const boost::property_tree::ptree& configuration);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

private:

	void
	sendPacket(const Packet& packet);

	void
	receivePacket(const Packet& packet);

	void
	tryConnectChannelMixing();

	ObjectHandle<IPC> ipc_;
	ObjectHandle<SerialIDC> idc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;

	Subscription flightControlSubscription_;
	Subscription missionControlSubscription_;
	Subscription channelMixingSubscription_;
	Publisher flightControlPublisher_;
	Publisher missionControlPublisher_;
	Publisher apiPublisher_;

	std::mutex senderMutex_;
	Sender sender_;

	std::string serialPort_;
};

#endif /* UAVAP_COMMUNICATION_COMM_SERIALCOMM_SERIALCOMM_H_MM_H_ */
