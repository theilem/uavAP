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
 * RectanguloidCondition.h
 *
 *  Created on: Aug 02, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RECTANGULOIDCONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RECTANGULOIDCONDITION_H_

#include <boost/signals2.hpp>
#include <uavAP/MissionControl/Geofencing/Rectanguloid.h>

#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/ConditionManager/ICondition.h"

class ConditionManager;

class RectanguloidCondition: public ICondition
{
public:

	static constexpr const char * const typeId = "rectanguloid";

	RectanguloidCondition();

	RectanguloidCondition(const Rectanguloid& rectanguloid);

	static std::shared_ptr<RectanguloidCondition>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	activate(ConditionManager* conditionManager, const ConditionTrigger& conditionTrigger) override;

	void
	deactivate() override;

	void
	onSensorData(const SensorData& data);

	enum Trigger
	{
		ENTER_RECTANGULOID,
		EXIT_RECTANGULOID
	};

private:

	boost::signals2::connection connection_;
	Rectanguloid rectanguloid_;
	ConditionTrigger trigger_;
	bool inTriggered_;
	bool outTriggered_;
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RECTANGULOIDCONDITION_H_ */
