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
 * RectanguloidCondition.cpp
 *
 *  Created on: Aug 02, 2018
 *      Author: simonyu
 */

#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/RectanguloidCondition.h"
#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlannerParams.h"

RectanguloidCondition::RectanguloidCondition() :
		connection_(), rectanguloid_(), trigger_(), inTriggered_(false), outTriggered_(
				false)
{
}

RectanguloidCondition::RectanguloidCondition(const Rectanguloid& rectanguloid) :
		rectanguloid_(rectanguloid), inTriggered_(false), outTriggered_(false)
{
	rectanguloid_.majorSideOrientation *=( M_PI / 180);
}

std::shared_ptr<RectanguloidCondition>
RectanguloidCondition::create(const Configuration& config)
{
	auto rectanguloidCondition = std::make_shared<RectanguloidCondition>();

	if (!rectanguloidCondition->configure(config))
	{
		CPSLOG_ERROR << "RectanguloidCondition: Failed to Load Config.";
	}

	return rectanguloidCondition;
}

bool
RectanguloidCondition::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	ParameterRef<Rectanguloid> rect(rectanguloid_, {}, "rectanguloid", true);

	pm & rect;

	rectanguloid_.majorSideOrientation *= (M_PI / 180);

	return pm.map();
}

void
RectanguloidCondition::activate(ConditionManager* conditionManager,
		const ConditionTrigger& conditionTrigger)
{
	connection_ = conditionManager->subscribeOnSensorData(
			std::bind(&RectanguloidCondition::onSensorData, this, std::placeholders::_1));
	trigger_ = conditionTrigger;
}

void
RectanguloidCondition::deactivate()
{
	connection_.disconnect();
}

void
RectanguloidCondition::onSensorData(const SensorData& data)
{
	double majorLengthHalf = rectanguloid_.majorSideLength / 2;
	double minorLengthHalf = rectanguloid_.minorSideLength / 2;
	double heightHalf = rectanguloid_.height / 2;
	Vector3 position = data.position;
	const Vector3& center = rectanguloid_.center;
	Vector3 distanceToCenter = position - center;
	Vector2 distanceToCenter2D = distanceToCenter.head(2);
	Vector2 distanceToCenter2DRotated = rotate2Drad(distanceToCenter2D,
			- rectanguloid_.majorSideOrientation);

	bool inRectanguloid = (std::fabs(distanceToCenter2DRotated.x()) < majorLengthHalf)
			&& (std::fabs(distanceToCenter2DRotated.y()) < minorLengthHalf)
			&& (std::fabs(distanceToCenter.z()) < heightHalf);

	if (inRectanguloid && !inTriggered_)
	{
		trigger_(Trigger::ENTER_RECTANGULOID);
		inTriggered_ = true;
		outTriggered_ = false;
	}
	else if (!inRectanguloid && !outTriggered_)
	{
		trigger_(Trigger::EXIT_RECTANGULOID);
		inTriggered_ = false;
		outTriggered_ = true;
	}
}
