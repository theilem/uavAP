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
 *
 */
#include <boost/property_tree/json_parser.hpp>
#include "uavAP/API/ap_ext/ap_ext.h"
#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include <string>

#define SERVO_MIN 3600
#define SERVO_MAX 8400
#define SERVO_RANGE_HALF 2400

static ApExtManager* apManager = nullptr;
static std::string configPath = "/usr/local/config/alvolo.json";

int
ap_ext_setup()
{
	APLOG_DEBUG << "Setup of ap_ext";
	if (apManager)
	{
		APLOG_ERROR << "Setup already executed.";
		return -1;
	}

	boost::property_tree::ptree interfaceConf;
	try
	{
		boost::property_tree::ptree conf;
		boost::property_tree::read_json(configPath, conf);
		PropertyMapper pm(conf);
		if (!pm.add("interface", interfaceConf, true))
			return -1;

	} catch (boost::property_tree::json_parser_error& err)
	{
		APLOG_ERROR << "Configuration file problem: " << err.what();
		return -1;
	}

	apManager = new ApExtManager();
	if (!apManager->configure(interfaceConf))
	{
		APLOG_ERROR << "Configuration of ApExtManager failed. Further behavior undefined.";
		return -1;
	}

	APLOG_DEBUG << "Successful setup.";

	return 0;
}

int
ap_ext_sense(const struct data_sample_t * sample)
{

	if (!apManager)
	{
		APLOG_ERROR << "Setup not called before sensing.";
		return -1;
	}

	return apManager->ap_sense(sample);
}

int
ap_ext_actuate(unsigned long * pwm, unsigned int num_channels)
{
	if (!apManager)
	{
		APLOG_ERROR << "Setup not called before actuating.";
		return -1;
	}

	return apManager->ap_actuate(pwm, num_channels);
}

int
ap_ext_teardown()
{
	APLOG_DEBUG << "Ap_ext teardown called";
	if (!apManager)
	{
		APLOG_ERROR << "Teardown called without setup.";
		return -1;
	}
	delete apManager;
	return 0;
}

int
ap_ext_ctrl(int* cmd)
{
	return 0;
}

ApExtManager*
getApExtManager()
{
	return apManager;
}

void
setConfigPath(const std::string& path)
{
	configPath = path;
}
