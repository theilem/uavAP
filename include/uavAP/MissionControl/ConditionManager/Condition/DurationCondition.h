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
 * DurationCondition.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_DURATIONCONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_DURATIONCONDITION_H_

#include <boost/property_tree/ptree.hpp>

#include "uavAP/MissionControl/ConditionManager/ICondition.h"
#include <cpsCore/Utilities/Scheduler/Event.h>

class ConditionManager;

class DurationCondition: public ICondition
{
public:

	static constexpr const char * const typeId = "duration";

	DurationCondition() = default;

	static std::shared_ptr<DurationCondition>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	activate(ConditionManager* conditionManager, const ConditionTrigger& conditionTrigger) override;

	void
	deactivate() override;

private:

	Event event_;
	Duration duration_;
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_DURATIONCONDITION_H_ */
