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
 * SerialHandler.cpp
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */
#include <uavAP/Core/IDC/Header/NetworkHeader.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>
#include "uavAP/Core/IDC/NetworkLayer/Serial/SerialHandler.h"

SerialHandler::SerialHandler(const SerialNetworkParams& params) :
		io_(), serial_(io_, params.serialPort), delim_(
				params.delimiterString[params.delimiterString.size() - 1]), delimString_(
				params.delimiterString), useCRC_(params.useCRC), direction_(params.direction), handlerCanceled_(
				false)
{
	serial_.set_option(boost::asio::serial_port_base::baud_rate(params.baudRate));
	if (params.flowControl)
		serial_.set_option(boost::asio::serial_port_base::flow_control(*params.flowControl));

	if (::tcflush(serial_.lowest_layer().native_handle(), TCIFLUSH) != 0)
	{
		APLOG_WARN << "Cannot flush serial port.";
	}

	if (direction_ != SerialDirection::SEND)
		boost::asio::async_read_until(serial_, inBuffer_, delim_,
				std::bind(&SerialHandler::receive, this, std::placeholders::_1,
						std::placeholders::_2));
}

boost::signals2::connection
SerialHandler::subscribeOnPackets(const OnPacket::slot_type& slot)
{
	if (direction_ == SerialDirection::SEND)
	{
		APLOG_ERROR << "Handler only configured to send. Will not receive.";
		return boost::signals2::connection();
	}
	return onPacket_.connect(slot);
}

bool
SerialHandler::sendPacket(const Packet& packet)
{
	if (direction_ == SerialDirection::RECEIVE)
	{
		APLOG_ERROR << "Handler only configured to receive. Will not send.";
		return false;
	}

	boost::system::error_code error;
	std::string message = packet.getBuffer();

	if (useCRC_)
	{
		NetworkHeader header;
		header.crc = packet.getCRC16();
		message.insert(0, dp::serialize(header).getBuffer());
		APLOG_TRACE << "Send Header CRC: " << header.crc;
	}

	message += delimString_;

	std::ostream os(&outBuffer_);
	os << message;

	boost::asio::async_write(serial_, outBuffer_,
			boost::bind(&SerialHandler::sendStatus, this, boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

	return true;
}

void
SerialHandler::startHandler()
{
	while (!handlerCanceled_.load())
	{
		io_.run();
		io_.reset();
	}
	APLOG_DEBUG << "Serial handler canceled" << std::endl;
}

void
SerialHandler::receive(const boost::system::error_code& err, std::size_t bytes_transferred)
{
	if (handlerCanceled_.load())
		return;
	if (err)
	{
		APLOG_ERROR << "Reception error: " << err.message();
		return;
	}
	APLOG_TRACE << "Serial received " << bytes_transferred << "bytes";
	std::istream read_buffer(&inBuffer_);
	std::string line;
	std::getline(read_buffer, line, delim_);

	boost::asio::async_read_until(serial_, inBuffer_, delim_,
			std::bind(&SerialHandler::receive, this, std::placeholders::_1, std::placeholders::_2));

	std::size_t delimSize = delimString_.size();
	std::string packetString;
	if (delimSize == 1)
	{
		packetString = line;
	}
	else
	{

		packetBuffer_ << line << delim_;

		packetString = packetBuffer_.str();

		if (packetString.size() >= delimString_.size()
				&& delimString_.compare(
						packetString.substr(packetString.size() - delimSize,
								packetString.size() - 1)) == 0)
		{
			//End of packet
			packetString.erase(packetString.size() - delimSize, packetString.size() - 1);
			packetBuffer_.clear();
			packetBuffer_.str(std::string());
		}
		else
			return;
	}

	Packet packet(packetString);

	if (useCRC_)
	{
		NetworkHeader header = dp::extract<NetworkHeader>(packet);
		uint16_t crc = packet.getCRC16();
		APLOG_TRACE << "Receive Header CRC: " << header.crc << ", Packet CRC: " << crc;
		if (header.crc != crc)
		{
			APLOG_ERROR << "CRC does not match";
			return;
		}
	}

	onPacket_(packet);
}

void
SerialHandler::sendStatus(const boost::system::error_code& ec, std::size_t bytes)
{
	APLOG_TRACE << "Serial " << bytes << " sent";
	if (ec)
	{
		APLOG_ERROR << "Error while sending: " << ec.message();
	}
}

void
SerialHandler::cancelHandler()
{
	handlerCanceled_.store(true);
	io_.stop();
}
