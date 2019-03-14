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
#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/DataPresentation/DataFilter/DataFilterFactory.h"
#include "uavAP/MissionControl/ConditionManager/ICondition.h"
#include "uavAP/MissionControl/ConditionManager/Condition/Relational.h"

class ConditionManager;

class SensorDataCondition : public ICondition
{
public:

	static constexpr const char * const typeId = "sensor_data";

	SensorDataCondition();

	static std::shared_ptr<SensorDataCondition>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

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

	boost::property_tree::ptree config_;
	boost::signals2::connection connection_;
	std::shared_ptr<IDataFilter> dataFilter_;
	DataFilterFactory dataFilterFactory_;
	ConditionTrigger trigger_;
	Sensor sensor_;
	Relational relational_;
	double tolerance_;
	bool useTolerance_;
	bool filterData_;
	bool dataFilterInitialized_;
};

template<typename Data>
inline bool
SensorDataCondition::evaluateSensorData(const Data& sensorData)
{
	PropertyMapper pm(config_);
	bool evaluation = false;
	Data threshold;

	pm.add<Data>("threshold", threshold, true);

	if (!pm.map())
	{
		std::string sensor = EnumMap<Sensor>::convert(sensor_);
		APLOG_ERROR << "SensorDataCondition: Failed to Parse Threshold for Sensor " << sensor;
		return false;
	}

	switch(relational_)
	{
	case Relational::EQUAL_TO:
	{
		evaluation = (sensorData == threshold);
		break;
	}
	case Relational::NOT_EQUAL_TO:
	{
		evaluation = (sensorData != threshold);
		break;
	}
	case Relational::GREATER_THAN:
	{
		evaluation = (sensorData > threshold);
		break;
	}
	case Relational::LESS_THAN:
	{
		evaluation = (sensorData < threshold);
		break;
	}
	case Relational::GREATER_THAN_EQUAL_TO:
	{
		evaluation = (sensorData >= threshold);
		break;
	}
	case Relational::LESS_THAN_EQUAL_TO:
	{
		evaluation = (sensorData <= threshold);
		break;
	}
	case Relational::INVALID:
	{
		std::string relational = EnumMap<Relational>::convert(relational_);
		APLOG_ERROR << "SensorDataCondition: Invalid Relational " << relational;
		break;
	}
	default:
	{
		std::string relational = EnumMap<Relational>::convert(relational_);
		APLOG_ERROR << "SensorDataCondition: Unknown Relational " << relational;
		break;
	}
	}

	return evaluation;
}

template<typename Data>
inline bool
SensorDataCondition::evaluateSensorDataWithTolerance(const Data& sensorData)
{
	PropertyMapper pm(config_);
	bool evaluation = false;
	Data threshold;
	Data thresholdUpper;
	Data thresholdLower;

	pm.add<Data>("threshold", threshold, true);

	if (!pm.map())
	{
		std::string sensor = EnumMap<Sensor>::convert(sensor_);
		APLOG_ERROR << "SensorDataCondition: Failed to Parse Threshold for Sensor " << sensor;
		return false;
	}

	thresholdUpper = threshold + tolerance_;
	thresholdLower = threshold - tolerance_;

	switch(relational_)
	{
	case Relational::EQUAL_TO:
	{
		evaluation = ((sensorData <= thresholdUpper) && (sensorData >= thresholdLower));
		break;
	}
	case Relational::NOT_EQUAL_TO:
	{
		evaluation = ((sensorData > thresholdUpper) || (sensorData < thresholdLower));
		break;
	}
	case Relational::GREATER_THAN:
	{
		evaluation = (sensorData > thresholdUpper);
		break;
	}
	case Relational::LESS_THAN:
	{
		evaluation = (sensorData < thresholdLower);
		break;
	}
	case Relational::GREATER_THAN_EQUAL_TO:
	{
		evaluation = (sensorData >= thresholdUpper);
		break;
	}
	case Relational::LESS_THAN_EQUAL_TO:
	{
		evaluation = (sensorData <= thresholdLower);
		break;
	}
	case Relational::INVALID:
	{
		std::string relational = EnumMap<Relational>::convert(relational_);
		APLOG_ERROR << "SensorDataCondition: Invalid Relational " << relational;
		break;
	}
	default:
	{
		std::string relational = EnumMap<Relational>::convert(relational_);
		APLOG_ERROR << "SensorDataCondition: Unknown Relational " << relational;
		break;
	}
	}

	return evaluation;
}

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_ */
