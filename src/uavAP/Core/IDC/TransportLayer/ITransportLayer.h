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
 * ITransportLayer.h
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_TRANSPORTLAYER_ITRANSPORTLAYER_H_
#define UAVAP_CORE_IDC_TRANSPORTLAYER_ITRANSPORTLAYER_H_
#include <boost/signals2.hpp>

class Packet;

class ITransportLayer
{
public:

	virtual
	~ITransportLayer() = default;

	virtual bool
	send(const std::string& dest, const Packet& packet, bool ack = false) = 0;

	using OnPacket = boost::signals2::signal<void(const Packet&)>;

	virtual boost::signals2::connection
	subscribeOnPacket(const std::string& orig, const OnPacket::slot_type& slot) = 0;
};

#endif /* UAVAP_CORE_IDC_TRANSPORTLAYER_ITRANSPORTLAYER_H_ */
