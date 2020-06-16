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
 * FlightPlannerFactory.h
 *
 *  Created on: Jun 8, 2017
 *      Author: mircot
 */

#ifndef FLIGHTPLANNER_FLIGHTPLANNERFACTORY_H_
#define FLIGHTPLANNER_FLIGHTPLANNERFACTORY_H_

#include <memory>
#include <boost/property_tree/ptree.hpp>

#include <cpsCore/Framework/StaticFactory.h>
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/FilletGlobalPlanner/FilletGlobalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/SplineGlobalPlanner/SplineGlobalPlanner.h"
//
//class GlobalPlannerFactory: public Factory<IGlobalPlanner>
//{
//public:
//	GlobalPlannerFactory()
//	{
//		addCreator<FilletGlobalPlanner>();
//		addCreator<SplineGlobalPlanner>();
//	}
//};

using GlobalPlannerFactory = StaticFactory<IGlobalPlanner, false, SplineGlobalPlanner, FilletGlobalPlanner>;

#endif /* FLIGHTPLANNER_FLIGHTPLANNERFACTORY_H_ */
