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
 * AirplaneLocalPlannerImpl.h
 *
 *  Created on: Aug 18, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_DETAIL_AIRPLANELOCALPLANNERIMPL_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_DETAIL_AIRPLANELOCALPLANNERIMPL_H_
#include "uavAP/Core/PropertyMapper/Configuration.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/ILinearPlannerImpl.h"

struct IPathSection;

class AirplaneLocalPlannerImpl: public ILinearPlannerImpl
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	AirplaneLocalPlannerImpl();

	bool
	configure(const Configuration& config) override;

	bool
	tuneParams(const LocalPlannerParams& params) override;

	ControllerTarget
	evaluate(const Vector3& position, double heading, std::shared_ptr<IPathSection> section)
			override;

	LocalPlannerStatus
	getStatus() override;

private:

	AirplaneLinearLocalPlannerParams params_;

	double headingTarget_;
};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_DETAIL_AIRPLANELOCALPLANNERIMPL_H_ */
