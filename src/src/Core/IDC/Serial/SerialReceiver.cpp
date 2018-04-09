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
 * SerialReceiver.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include <boost/asio/read_until.hpp>
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/IDC/Serial/SerialReceiver.h"
#include "uavAP/Core/Logging/APLogger.h"

SerialReceiver::SerialReceiver(const SerialIDCParams& params, const OnPacket& handle) :
		io_(),
		serial_(io_, params.serialPort),
		delim_(params.delimiterString[params.delimiterString.size() - 1]),
		delimString_(params.delimiterString),
		onPacket_(handle)
{
	serial_.set_option(boost::asio::serial_port_base::baud_rate(params.baudRate));
	if (params.flowControl)
		serial_.set_option(boost::asio::serial_port_base::flow_control(*params.flowControl));

	if (::tcflush(serial_.lowest_layer().native_handle(), TCIFLUSH) != 0)
	{
		APLOG_WARN << "Cannot flush serial port.";
	}

	boost::asio::async_read_until(serial_, buffer_, delim_,
			std::bind(&SerialReceiver::receive, this, std::placeholders::_1,
					std::placeholders::_2));
}

void
SerialReceiver::startReceive()
{
	while (1)
	{
		io_.run();
	}
}

void
SerialReceiver::receive(const boost::system::error_code& err, std::size_t bytes_transferred)
{
	if (err)
	{
		APLOG_ERROR << "Reception error: " << err.message();
		return;
	}
	APLOG_TRACE << "Serial received " << bytes_transferred << "bytes";
	std::istream read_buffer(&buffer_);
	std::string line;
	std::getline(read_buffer, line, delim_);

	boost::asio::async_read_until(serial_, buffer_, delim_,
			std::bind(&SerialReceiver::receive, this, std::placeholders::_1,
					std::placeholders::_2));

	std::size_t delimSize = delimString_.size();
	if (delimSize == 1)
	{
		onPacket_(Packet(line));
		return;
	}

	packetBuffer_ << line << delim_;

	std::string packetString = packetBuffer_.str();

	if (packetString.size() >= delimString_.size() && delimString_.compare(
			packetString.substr(packetString.size() - delimSize, packetString.size() - 1)) == 0)
	{
		//End of packet
		packetString.erase(packetString.size() - delimSize, packetString.size() - 1);
		packetBuffer_.clear();
		packetBuffer_.str(std::string());
		onPacket_(Packet(packetString));
	}
}
