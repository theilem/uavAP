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
 * SerialHandler.h
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_NETWORKLAYER_SERIAL_SERIALHANDLER_H_
#define UAVAP_CORE_IDC_NETWORKLAYER_SERIAL_SERIALHANDLER_H_

#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <uavAP/Core/DataPresentation/Packet.h>
#include <uavAP/Core/IDC/NetworkLayer/Serial/SerialNetworkParams.h>

class SerialHandler
{
public:

	SerialHandler(const SerialNetworkParams& params);

	~SerialHandler();

	using OnPacket = boost::signals2::signal<void(const Packet&)>;

	boost::signals2::connection
	subscribeOnPackets(const OnPacket::slot_type& slot);

	bool
	sendPacket(const Packet& packet);

	void
	startHandler();

	void
	cancelHandler();

private:

	void
	receive(const boost::system::error_code& err, std::size_t bytes_transferred);

	void
	sendStatus(const boost::system::error_code& ec, std::size_t bytes);

	boost::asio::io_service io_;
	boost::asio::serial_port serial_;

	char delim_;
	std::string delimString_;
	bool useCRC_;
	SerialDirection direction_;
	bool sendBlocking_;

	std::stringstream packetBuffer_;


	boost::asio::streambuf inBuffer_;
	boost::asio::streambuf outBuffer_;

	OnPacket onPacket_;
	std::atomic_bool handlerCanceled_;

};

#endif /* UAVAP_CORE_IDC_NETWORKLAYER_SERIAL_SERIALHANDLER_H_ */
