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
 * LocalPlannerParams.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: mircot
 */
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/LocalPlannerParams.h"

bool
HelicopterLocalPlannerParams::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper propertyMapper(config);
	propertyMapper.add("k_altitude", kAltitude_, true);
	propertyMapper.add("k_velocity", kVelocity_, true);
	propertyMapper.add("k_heading", kHeading_, true);
	propertyMapper.add("k_yaw_rate", kYawRate_, true);

	return propertyMapper.map();
}
