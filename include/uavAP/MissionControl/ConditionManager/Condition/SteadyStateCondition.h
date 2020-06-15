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
 * SteadyStateCondition.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_STEADYSTATECONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_STEADYSTATECONDITION_H_

#include <boost/signals2.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cpsCore/Utilities/Packet.h>

#include "uavAP/FlightAnalysis/StateAnalysis/Metrics.h"
#include "uavAP/MissionControl/ConditionManager/ICondition.h"

class ConditionManager;
class DataPresentation;

class SteadyStateCondition: public ICondition
{
public:

	static constexpr const char * const typeId = "steady_state";

	SteadyStateCondition();

	static std::shared_ptr<SteadyStateCondition>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	activate(ConditionManager* conditionManager, const ConditionTrigger& conditionTrigger) override;

	void
	deactivate() override;

	void
	onSteadyState(const Packet& packet);

private:

	void
	minDuration();

	boost::signals2::connection connection_;
	bool steadyState_;
	ConditionTrigger trigger_;
	Event event_;
	Duration minimumDuration_;

	bool afterMinimumDuration_;

	std::shared_ptr<DataPresentation> dp_;
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_STEADYSTATECONDITION_H_ */
