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
 * GeofencingCondition.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/MissionControl/ConditionManager/Condition/GeofencingCondition.h>
#include <uavAP/MissionControl/ConditionManager/ConditionManager.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>

GeofencingCondition::GeofencingCondition() :
		rollRate_(0), rollMax_(0), precision_(16), g_(9.81), yawCenterLeft_(0), yawCenterRight_(0)
{
	acb_init(betaCompleteLeft_);
	acb_init(betaCompleteRight_);
	acb_init(aLeft_);
	acb_init(aRight_);
	acb_init(b_);
	acb_init(query_);
	acb_init(queryRes_);
	acb_init(centerLeft_);
	acb_init(centerRight_);

	arb_init(factor_);
	arb_init(real_);
	arb_init(imag_);

	acb_set_d(b_, 0.5);
}



GeofencingCondition::~GeofencingCondition()
{
	deactivate();

	acb_clear(betaCompleteLeft_);
	acb_clear(betaCompleteRight_);
	acb_clear(aLeft_);
	acb_clear(aRight_);
	acb_clear(b_);
	acb_clear(query_);
	acb_clear(queryRes_);
	acb_clear(centerLeft_);
	acb_clear(centerRight_);

	arb_clear(factor_);
	arb_clear(real_);
	arb_clear(imag_);
}

std::shared_ptr<GeofencingCondition>
GeofencingCondition::create(const Configuration& config)
{
	auto cond = std::make_shared<GeofencingCondition>();
	cond->configure(config);
	return cond;
}

bool
GeofencingCondition::configure(const Configuration& config)
{
	PropertyMapper pm(config);
	pm.add<double>("roll_rate", rollRate_, true);
	pm.add<double>("roll_max", rollMax_, true);
	pm.add<long int>("precision", precision_, true);
	pm.add<Polygon>("polygon", polygon_, true);

	return pm.map();
}

void
GeofencingCondition::activate(ConditionManager* conditionManager,
		const ConditionTrigger& conditionTrigger)
{
	trigger_ = conditionTrigger;
	connection_ = conditionManager->subscribeOnSensorData(
			std::bind(&GeofencingCondition::onSensorData, this, std::placeholders::_1));
}

void
GeofencingCondition::deactivate()
{
	connection_.disconnect();
}

void
GeofencingCondition::onSensorData(const SensorData& data)
{
	updateParameters(data);


	VehicleOneFrame frame(data.attitude.z(), data.position);
	for (const auto& it : polygon_.getEdges())
	{
		double angle = it.yaw;
		if (angle < 0)
			angle += M_PI; //Bring to [0, pi)



	}

}
