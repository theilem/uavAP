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
 * SingleMediaTransport.h
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_TRANSPORTLAYER_SINGLEMEDIATRANSPORT_SINGLEMEDIATRANSPORT_H_
#define UAVAP_CORE_IDC_TRANSPORTLAYER_SINGLEMEDIATRANSPORT_SINGLEMEDIATRANSPORT_H_
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/IDC/TransportLayer/ITransportLayer.h>
#include <uavAP/Core/IDC/TransportLayer/SingleMediaTransport/SMTParams.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

class SerialNetworkLayer;

class SingleMediaTransport: public ITransportLayer,
		public IAggregatableObject,
		public IRunnableObject
{
public:

	static constexpr TypeId typeId = "single";

	static std::shared_ptr<SingleMediaTransport>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	send(const std::string& dest, const Packet& packet, bool ack = false) override;

	boost::signals2::connection
	subscribeOnPacket(const std::string& orig, const OnPacket::slot_type& slot) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	ObjectHandle<SerialNetworkLayer> serialNetwork_;

	std::map<std::string, SMTParams> networkConfiguration_;

};

#endif /* UAVAP_CORE_IDC_TRANSPORTLAYER_SINGLEMEDIATRANSPORT_SINGLEMEDIATRANSPORT_H_ */
