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
 * SensorDataCondition.cpp
 *
 *  Created on: Mar 12, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SensorDataCondition.h"

// TODO: Finish sensor switch, Filter data (Initialize filter), Tolerance

SensorDataCondition::SensorDataCondition() :
		config_(), connection_(), dataFilter_(), dataFilterFactory_(), trigger_(), sensor_(), relational_(), tolerance_(
				0), useTolerance_(false), filterData_(false), dataFilterInitialized_(false)
{
}

std::shared_ptr<SensorDataCondition>
SensorDataCondition::create(const boost::property_tree::ptree& config)
{
	auto sensorDataCondition = std::make_shared<SensorDataCondition>();

	if (!sensorDataCondition->configure(config))
	{
		APLOG_ERROR << "SensorDataCondition: Failed to Load Config.";
	}

	return sensorDataCondition;
}

bool
SensorDataCondition::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree dataFilterTree;

	config_ = config;

	if (!pm.add<bool>("filter_data", filterData_, false))
	{
		filterData_ = false;
	}

	if (!pm.add<bool>("use_tolerance", useTolerance_, false))
	{
		useTolerance_ = false;
	}

	pm.addEnum<Sensor>("sensor", sensor_, true);
	pm.addEnum<Relational>("relational", relational_, true);

	if (useTolerance_ && !pm.add<double>("tolerance", tolerance_, false))
	{
		useTolerance_ = false;
	}

	if (filterData_ && pm.add("data_filter", dataFilterTree, false))
	{
		dataFilter_ = dataFilterFactory_.create(dataFilterTree);
	}
	else
	{
		filterData_ = false;
	}

	dataFilterInitialized_ = false;

	return pm.map();
}

void
SensorDataCondition::activate(ConditionManager* conditionManager,
		const ConditionTrigger& conditionTrigger)
{
	connection_ = conditionManager->subscribeOnSensorData(
			std::bind(&SensorDataCondition::onSensorData, this, std::placeholders::_1));
	trigger_ = conditionTrigger;
}

void
SensorDataCondition::deactivate()
{
	connection_.disconnect();
}

void
SensorDataCondition::onSensorData(const SensorData& data)
{
	bool evaluation = false;

	switch (sensor_)
	{
	case Sensor::POSITION_X:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filterSensorData(data.position.x()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.position.x()));
		}

		break;
	}
	case Sensor::POSITION_Y:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filterSensorData(data.position.y()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.position.y()));
		}

		break;
	}
	case Sensor::POSITION_Z:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filterSensorData(data.position.z()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.position.z()));
		}

		break;
	}
	/* FINISH ALL SENSORS */
	case Sensor::TIMESTAMP:
	{
		APLOG_ERROR << "SensorDataCondition: Timestamp Cannot Be Evaluated.";
		break;
	}
	case Sensor::SEQUENCE_NR:
	{
		useTolerance_ = false;
		filterData_ = false;
		evaluation = evaluateSensorData<uint32_t>(data.sequenceNr);
		break;
	}
	case Sensor::HAS_GPS_FIX:
	{
		useTolerance_ = false;
		filterData_ = false;
		evaluation = evaluateSensorData<bool>(data.hasGPSFix);
		break;
	}
	case Sensor::AUTOPILOT_ACTIVE:
	{
		useTolerance_ = false;
		filterData_ = false;
		evaluation = evaluateSensorData<bool>(data.autopilotActive);
		break;
	}
	/* FINISH ALL SENSORS */
	case Sensor::INVALID:
	{
		std::string sensor = EnumMap<Sensor>::convert(sensor_);
		APLOG_ERROR << "SensorDataCondition: Invalid Sensor " << sensor;
		break;
	}
	default:
	{
		std::string sensor = EnumMap<Sensor>::convert(sensor_);
		APLOG_ERROR << "SensorDataCondition: Unknown Sensor " << sensor;
		break;
	}
	}

	if (evaluation)
	{
		trigger_(0);
	}
}

double
SensorDataCondition::filterSensorData(const double& sensorData)
{
	double sensorDataFiltered = sensorData;

	if (filterData_)
	{
		if (!dataFilterInitialized_)
		{
			dataFilter_->initialize(sensorData);
			dataFilterInitialized_ = true;
		}

		dataFilter_->filterData(sensorData);
		sensorDataFiltered = dataFilter_->getFilteredData();
	}

	return sensorDataFiltered;
}
