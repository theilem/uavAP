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
 * ConditionFactory.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_CONDITIONFACTORY_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_CONDITIONFACTORY_H_

#include <cpsCore/Framework/StaticFactory.h>
#include "uavAP/MissionControl/ConditionManager/Condition/DurationCondition.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SteadyStateCondition.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SensorDataCondition.h"

using ConditionFactory = StaticFactory<ICondition, false, DurationCondition, SteadyStateCondition, SensorDataCondition>;

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_CONDITIONFACTORY_H_ */
