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
 * SerialSenderImpl.h
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_SERIAL_SERIALSENDERIMPL_H_
#define UAVAP_CORE_IDC_SERIAL_SERIALSENDERIMPL_H_
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include "uavAP/Core/IDC/ISenderImpl.h"

struct SerialIDCParams;
class Packet;

class SerialSenderImpl: public ISenderImpl
{
public:

	SerialSenderImpl(const SerialIDCParams& params);

	bool
	sendPacket(const Packet& packet) override;

private:

	void
	sendStatus(const boost::system::error_code& ec, std::size_t bytes);

	boost::asio::io_service io_;
	boost::asio::serial_port serial_;
	std::string delimString_;

	boost::asio::streambuf buffer_;
	SerialIDCParams params_;

};


#endif /* UAVAP_CORE_IDC_SERIAL_SERIALSENDERIMPL_H_ */
