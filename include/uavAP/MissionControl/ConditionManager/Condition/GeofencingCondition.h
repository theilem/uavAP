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
 * GeofencingCondition.h
 *
 *  Created on: Aug 14, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_GEOFENCINGCONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_GEOFENCINGCONDITION_H_
#include <boost/property_tree/ptree.hpp>
#include <boost/signals2/connection.hpp>
#include <uavAP/Core/SensorData.h>
#include "uavAP/MissionControl/ConditionManager/ICondition.h"
#include <memory>

#include <acb_hypgeom.h>
#include <uavAP/MissionControl/Polygon.h>

class GeofencingCondition: public ICondition
{

public:

	static constexpr const char * const typeId = "geofencing";

	GeofencingCondition();

	~GeofencingCondition();

	static std::shared_ptr<GeofencingCondition>
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
		TURN_RIGHT, TURN_LEFT
	};

private:

	boost::signals2::connection connection_;
	ConditionTrigger trigger_;

	Polygon polygon_;

};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_GEOFENCINGCONDITION_H_ */
