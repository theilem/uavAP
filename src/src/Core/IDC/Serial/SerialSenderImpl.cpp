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
 * SerialSenderImpl.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include <boost/asio/buffer.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IDC/Serial/SerialIDCParams.h"
#include "uavAP/Core/IDC/Serial/SerialSenderImpl.h"
#include "uavAP/Core/Logging/APLogger.h"

SerialSenderImpl::SerialSenderImpl(const SerialIDCParams& params) :
		io_(),
		serial_(io_, params.serialPort),
		delimString_(params.delimiterString),
		params_(params)
{
	serial_.set_option(boost::asio::serial_port_base::baud_rate(params.baudRate));
	if (params.flowControl)
		serial_.set_option(boost::asio::serial_port_base::flow_control(*params.flowControl));
}

bool
SerialSenderImpl::sendPacket(const Packet& packet)
{
	boost::system::error_code error;
	std::string message = packet.getBuffer();
	message += delimString_;

	boost::asio::async_write(serial_, boost::asio::buffer(message.data(), message.size()),
			boost::bind(&SerialSenderImpl::sendStatus, this, boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
	io_.run();
	io_.reset();
	return true;
}

void
SerialSenderImpl::sendStatus(const boost::system::error_code& ec, std::size_t bytes)
{
	APLOG_TRACE << "Serial " << bytes << " sent";
	if (ec)
	{
		APLOG_ERROR << "Error while sending to " << params_.serialPort << ": " << ec.message();
	}
}
