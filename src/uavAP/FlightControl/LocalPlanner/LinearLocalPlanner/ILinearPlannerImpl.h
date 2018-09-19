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
 * ILinearPlannerImpl.h
 *
 *  Created on: Aug 18, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_ILINEARPLANNERIMPL_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_ILINEARPLANNERIMPL_H_
#include <boost/property_tree/ptree.hpp>
#include <boost/variant/variant.hpp>
#include "uavAP/Core/protobuf/messages/LocalPlanner.pb.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"

class LinearLocalPlanner;
class LocalPlannerParams;
struct IPathSection;

class ILinearPlannerImpl
{
public:

	virtual
	~ILinearPlannerImpl() = default;

	virtual bool
	configure(const boost::property_tree::ptree& config) = 0;

	virtual bool
	tuneParams(const LocalPlannerParams& params) = 0;

	virtual LocalPlannerStatus
	getStatus() = 0;

	virtual ControllerTarget
	evaluate(const Vector3& position, double heading, std::shared_ptr<IPathSection> section) = 0;
};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_ILINEARPLANNERIMPL_H_ */
