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
 * @file SerialIDCParams.h
 * @brief Defines SerialIDCParams
 * @date Jul 31, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */
#ifndef UAVAP_CORE_IDC_SERIAL_SERIALIDCPARAMS_H_
#define UAVAP_CORE_IDC_SERIAL_SERIALIDCPARAMS_H_
#include "uavAP/Core/IDC/IIDCParams.h"
#include <string>
#include <boost/asio/serial_port_base.hpp>
#include <boost/optional/optional.hpp>

/**
 * @brief Parameters used for SerialIDC
 */
struct SerialIDCParams: public IIDCParams
{
	std::string serialPort; //!< Serial port name e.g. "/dev/ttyUSB0"
	unsigned int baudRate; //!< Baud rate for serial port
	std::string delimiterString; //!< End of transmission delimiter string
	boost::optional<boost::asio::serial_port_base::flow_control::type> flowControl; //!< optional flow control setting

	SerialIDCParams(const std::string& port, unsigned int baud_rate, std::string delim);

};


#endif /* UAVAP_CORE_IDC_SERIAL_SERIALIDCPARAMS_H_ */
