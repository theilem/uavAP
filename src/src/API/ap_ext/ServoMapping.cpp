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
 * ServoMapping.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: mircot
 */
#include "uavAP/API/ap_ext/ServoMapping.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

ServoMapping::ServoMapping()
{
}

bool
ServoMapping::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	int numChannels = 0;
	boost::property_tree::ptree servoConf;
	pm.add("num_output_channel", numChannels, true);
	pm.add("servo", servoConf, true);

	if (!pm.map())
		return false;

	servos_ = std::vector<Servo>(numChannels, Servo());

	for (auto& it: servoConf)
	{
		PropertyMapper servoPm(it.second);
		int channel;
		if (servoPm.add("channel", channel, false))
		{
			if (channel < 0 || channel >= numChannels)
			{
				APLOG_ERROR << "Channel number " << channel << " invalid";
				return false;
			}
			if (!servos_.at(channel).configure(it.second))
			{
				APLOG_ERROR << "Servo config for channel " << channel << " failed";
				return false;
			}
		}
	}

	return true;
}

std::vector<unsigned long>
ServoMapping::map(std::vector<double> val)
{
	if (val.size() != servos_.size())
	{
		APLOG_ERROR << "Servo config does not match mapping size";
		return std::vector<unsigned long>(val.size(), 6000);
	}

	std::vector<unsigned long> output;
	unsigned int i = 0;
	for (auto& it : val)
	{
		output.push_back(servos_[i].map(it));
		++i;
	}
	return output;
}

std::vector<double>
ServoMapping::unmap(std::vector<unsigned long> val)
{
	if (val.size() != servos_.size())
	{
		APLOG_ERROR << "Servo config does not match mapping size";
		return std::vector<double>(val.size(), 0);
	}

	std::vector<double> output;
	unsigned int i = 0;
	for (auto& it : val)
	{
		output.push_back(servos_[i].unmap(it));
		++i;
	}
	return output;
}
