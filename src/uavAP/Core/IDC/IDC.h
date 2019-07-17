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
 * IDC.h
 *
 *  Created on: Jul 28, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_IDC_H_
#define UAVAP_CORE_IDC_IDC_H_
#include <uavAP/Core/IDC/IDCSender.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <boost/signals2.hpp>

class INetworkLayer;
class ITransportLayer;
class Packet;

class IDC: public IAggregatableObject, public IRunnableObject
{

public:

	static constexpr TypeId typeId = "idc";

	static std::shared_ptr<IDC>
	create(const Configuration& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	bool
	sendPacket(const std::string& id, const Packet& packet, bool ack = false);

	using OnPacket = boost::signals2::signal<void(const Packet&)>;

	boost::signals2::connection
	subscribeOnPacket(const std::string& id, const OnPacket::slot_type& handle);

	IDCSender
	createSender(const std::string& id);

private:

	ObjectHandle<INetworkLayer> network_;
	ObjectHandle<ITransportLayer> transport_;
	ObjectHandle<IDC> self_;

};

#endif /* UAVAP_CORE_IDC_IDC_H_ */
