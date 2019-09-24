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
#include <uavAP/Communication/Comm/IDCComm/IDCCommParams.h>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/IPC/Subscription.h>
#include <uavAP/Core/LockTypes.h>
#include <uavAP/Core/Scheduler/Event.h>

#include <uavAP/Core/Object/AggregatableObject.hpp>
#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>
#include "uavAP/Communication/Comm/IComm.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IDC/IDCSender.h"

class DataPresentation;
class IDC;
class IPC;

class IDCComm: public IComm,
		public AggregatableObject<IPC, IDC, DataPresentation>,
		public ConfigurableObject<IDCCommParams>,
		public IRunnableObject
{
public:

	static constexpr TypeId typeId = "idc";

	IDCComm() = default;

	bool
	run(RunStage stage) override;

private:

	void
	sendPacket(const Packet& packet);

	void
	receivePacket(const Packet& packet);

	void
	subscribeCallback(const Subscription& sub, Target target);

	std::vector<Subscription> subscriptions_;
	std::vector<Publisher<Packet>> publishers_;

	Mutex senderMutex_;
	IDCSender sender_;
	boost::signals2::connection idcConnection_;
};

#endif /* UAVAP_COMMUNICATION_COMM_IDCCOMM_IDCCOMM_H_MM_H_ */
