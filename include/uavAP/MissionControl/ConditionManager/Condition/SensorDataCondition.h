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
 * SensorDataCondition.h
 *
 *  Created on: Mar 12, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_

#include <boost/signals2.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/ConditionManager/ICondition.h"
#include "uavAP/MissionControl/ConditionManager/Condition/Relational.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SensorDataConditionParams.h"

class ConditionManager;

class SensorDataCondition : public ConfigurableObject<SensorDataConditionParams>, public ICondition
{
public:

	static constexpr const char* const typeId = "sensor_data";

	SensorDataCondition();

	void
	activate(ConditionManager* conditionManager, const ConditionTrigger& conditionTrigger) override;

	void
	deactivate() override;

	void
	onSensorData(const SensorData& data);

private:

	template<typename Data>
	bool
	evaluateSensorData(const Data& sensorData);

	template<typename Data>
	bool
	evaluateSensorDataWithTolerance(const Data& sensorData);

	double
	filterSensorData(const double& sensorData);

	Configuration config_;
	boost::signals2::connection connection_;
	ConditionTrigger trigger_;
};

template<typename Data>
inline bool
SensorDataCondition::evaluateSensorData(const Data& sensorData)
{
	switch (params.relational())
	{
		case Relational::EQUAL_TO:
		{
			return (sensorData == static_cast<Data>(params.threshold()));
		}
		case Relational::NOT_EQUAL_TO:
		{
			return (sensorData != static_cast<Data>(params.threshold()));
		}
		case Relational::GREATER_THAN:
		{
			return (sensorData > static_cast<Data>(params.threshold()));
		}
		case Relational::LESS_THAN:
		{
			return (sensorData < static_cast<Data>(params.threshold()));
		}
		case Relational::GREATER_THAN_EQUAL_TO:
		{
			return (sensorData >= static_cast<Data>(params.threshold()));
		}
		case Relational::LESS_THAN_EQUAL_TO:
		{
			return (sensorData <= static_cast<Data>(params.threshold()));
		}
		case Relational::INVALID:
		{
			CPSLOG_ERROR << "SensorDataCondition: Invalid Relational ";
			break;
		}
		default:
		{
			CPSLOG_ERROR << "SensorDataCondition: Unknown Relational ";
			break;
		}
	}

	return false;
}

template<typename Data>
inline bool
SensorDataCondition::evaluateSensorDataWithTolerance(const Data& sensorData)
{
	Data thresholdUpper = static_cast<Data>(params.threshold() + params.tolerance());
	Data thresholdLower = static_cast<Data>(params.threshold() - params.tolerance());

	switch (params.relational())
	{
		case Relational::EQUAL_TO:
		{
			return ((sensorData <= thresholdUpper) && (sensorData >= thresholdLower));
		}
		case Relational::NOT_EQUAL_TO:
		{
			return ((sensorData > thresholdUpper) || (sensorData < thresholdLower));
		}
		case Relational::GREATER_THAN:
		{
			return (sensorData > thresholdUpper);
		}
		case Relational::LESS_THAN:
		{
			return (sensorData < thresholdLower);
		}
		case Relational::GREATER_THAN_EQUAL_TO:
		{
			return (sensorData >= thresholdUpper);
		}
		case Relational::LESS_THAN_EQUAL_TO:
		{
			return (sensorData <= thresholdLower);
		}
		case Relational::INVALID:
		{
			CPSLOG_ERROR << "SensorDataCondition: Invalid Relational ";
			break;
		}
		default:
		{
			CPSLOG_ERROR << "SensorDataCondition: Unknown Relational ";
			break;
		}
	}

	return false;
}

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_ */
