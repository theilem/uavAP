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
#include <string>
#include <boost/asio/serial_port_base.hpp>
#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/IDC/NetworkLayer/INetworkParams.h>
#include <uavAP/Core/EnumMap.hpp>

enum SerialDirection
{
	BOTH = 0, SEND, RECEIVE
};

ENUMMAP_INIT(SerialDirection, {{SerialDirection::BOTH, "both"}, {SerialDirection::SEND, "send"},
		{SerialDirection::RECEIVE, "receive"}});
/**
 * @brief Parameters used for SerialIDC
 */
struct SerialNetworkParams: public INetworkParams
{
	std::string serialPort; //!< Serial port name e.g. "/dev/ttyUSB0"

	static constexpr unsigned int DEFAULT_BAUD_RATE = 115200;
	unsigned int baudRate = DEFAULT_BAUD_RATE; //!< Baud rate for serial port

	static constexpr auto DEFAULT_DELIMITER_STRING = "*-\n";
	std::string delimiterString = DEFAULT_DELIMITER_STRING; //!< End of transmission delimiter string
	boost::optional<boost::asio::serial_port_base::flow_control::type> flowControl; //!< optional flow control setting

	static constexpr bool DEFAULT_USE_CRC = false;
	bool useCRC = DEFAULT_USE_CRC; //!< Use cyclic redundancy check

	static constexpr SerialDirection DEFAULT_DIRECTION = SerialDirection::BOTH;
	SerialDirection direction = DEFAULT_DIRECTION;

	SerialNetworkParams() = default;

	SerialNetworkParams(const std::string& port, unsigned int baud_rate, std::string delim);

	bool
	configure(const boost::property_tree::ptree& config);

};

#endif /* UAVAP_CORE_IDC_SERIAL_SERIALIDCPARAMS_H_ */
