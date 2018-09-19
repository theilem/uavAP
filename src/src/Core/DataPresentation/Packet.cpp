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
 * Packet.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include "uavAP/Core/DataPresentation/Packet.h"
#include <boost/crc.hpp>

Packet::Packet(const std::string& buf) :
		buffer_(buf)
{
}

void
Packet::prepend(const std::string& str)
{
	buffer_.insert(0, str);
}

const std::string&
Packet::getBuffer() const
{
	return buffer_;
}

const char*
Packet::getStart()
{
	return buffer_.data();
}

std::size_t
Packet::getSize() const
{
	return buffer_.size();
}

std::string
Packet::consume(std::size_t length)
{
	if (length > buffer_.size())
		return std::string();

	std::string result = buffer_.erase(0, length);
	return result;
}

void
Packet::prepend(const Packet& str)
{
	prepend(str.getBuffer());
}

uint16_t
Packet::getCRC16() const
{
	boost::crc_ccitt_type crc;
	crc.process_bytes(static_cast<const void*>(buffer_.c_str()), getSize());
	return static_cast<uint16_t>(crc.checksum());
}
