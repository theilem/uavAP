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
 * SerialIDCParams.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include "uavAP/Core/IDC/NetworkLayer/Serial/SerialNetworkParams.h"


SerialNetworkParams::SerialNetworkParams(const std::string& port, unsigned int baud_rate, std::string delim) :
		serialPort(port), baudRate(baud_rate), delimiterString(delim)
{
}

bool
SerialNetworkParams::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	pm.add("serial_port", serialPort, true);
	pm.add<unsigned int>("baud_rate", baudRate, false);
	pm.add("delimiter_string", delimiterString, false);
	pm.add<bool>("use_crc", useCRC, false);
	pm.add<bool>("send_blocking", sendBlocking, false);

	std::string flowCTRL;
	if (pm.add("flow_controle", flowCTRL, false))
	{
		if (flowCTRL.compare("none") == 0)
		{
			flowControl = boost::asio::serial_port_base::flow_control::none;
		}
		else if (flowCTRL.compare("hw") == 0)
		{
			flowControl = boost::asio::serial_port_base::flow_control::hardware;
		}
		else if (flowCTRL.compare("sw") == 0)
		{
			flowControl = boost::asio::serial_port_base::flow_control::software;
		}
		else
		{
			APLOG_ERROR << "Flowcontrol type unknown, available: [none, hw, sw]";
		}
	}
	std::string dir;
	if (pm.add("direction", dir, false))
	{
		direction = EnumMap<SerialDirection>::convert(dir);
	}
	return pm.map();
}
