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
 * @file SerialReceiver.h
 * @brief
 * @date Jul 31, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_IDC_SERIAL_SERIALRECEIVER_H_
#define UAVAP_CORE_IDC_SERIAL_SERIALRECEIVER_H_
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IDC/Serial/SerialIDCParams.h"
#include <functional>
#include <sstream>

/**
 * @brief The SerialReceiver class
 */
class SerialReceiver
{
public:

    using OnPacket = std::function<void(const Packet&)>;

    /**
     * @brief SerialReceiver
     * @param params
     * @param handle
     */
	SerialReceiver(const SerialIDCParams& params, const OnPacket& handle);

	void
	startReceive();

private:

	void
	receive(const boost::system::error_code& err, std::size_t bytes_transferred);

	boost::asio::io_service io_;
	boost::asio::serial_port serial_;
	char delim_;
	std::string delimString_;

	std::stringstream packetBuffer_;

	boost::asio::streambuf buffer_;

	OnPacket onPacket_;
};


#endif /* UAVAP_CORE_IDC_SERIAL_SERIALRECEIVER_H_ */
