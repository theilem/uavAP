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
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.position.x()));
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
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.position.y()));
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
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.position.z()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.position.z()));
		}

		break;
	}
	case Sensor::VELOCITY_X:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.velocity.x()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.velocity.x()));
		}

		break;
	}
	case Sensor::VELOCITY_Y:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.velocity.y()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.velocity.y()));
		}

		break;
	}
	case Sensor::VELOCITY_Z:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.velocity.z()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.velocity.z()));
		}

		break;
	}
	case Sensor::ACCELERATION_X:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.acceleration.x()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.acceleration.x()));
		}

		break;
	}
	case Sensor::ACCELERATION_Y:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.acceleration.y()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.acceleration.y()));
		}

		break;
	}
	case Sensor::ACCELERATION_Z:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.acceleration.z()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.acceleration.z()));
		}

		break;
	}
	case Sensor::ATTITUDE_X:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.attitude.x()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.attitude.x()));
		}

		break;
	}
	case Sensor::ATTITUDE_Y:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.attitude.y()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.attitude.y()));
		}

		break;
	}
	case Sensor::ATTITUDE_Z:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.attitude.z()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.attitude.z()));
		}

		break;
	}
	case Sensor::ANGULAR_RATE_X:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.angularRate.x()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.angularRate.x()));
		}

		break;
	}
	case Sensor::ANGULAR_RATE_Y:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.angularRate.y()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.angularRate.y()));
		}

		break;
	}
	case Sensor::ANGULAR_RATE_Z:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.angularRate.z()));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.angularRate.z()));
		}

		break;
	}
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
	case Sensor::AIR_SPEED:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filterSensorData(data.airSpeed));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.airSpeed));
		}

		break;
	}
	case Sensor::GROUND_SPEED:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.groundSpeed));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.groundSpeed));
		}

		break;
	}
	case Sensor::ANGLE_OF_ATTACK:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.angleOfAttack));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.angleOfAttack));
		}

		break;
	}
	case Sensor::PROPULSION_POWER:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.propulsionPower));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.propulsionPower));
		}

		break;
	}
	case Sensor::CONSUMED_ENERGY:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.consumedEnergy));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.consumedEnergy));
		}

		break;
	}
	case Sensor::BATTERY_VOLTAGE:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.batteryVoltage));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.batteryVoltage));
		}

		break;
	}
	case Sensor::BATTERY_CURRENT:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(
					filterSensorData(data.batteryCurrent));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.batteryCurrent));
		}

		break;
	}
	case Sensor::THROTTLE:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filterSensorData(data.throttle));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.throttle));
		}

		break;
	}
	case Sensor::RPM:
	{
		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filterSensorData(data.rpm));
		}
		else
		{
			evaluation = evaluateSensorData<double>(filterSensorData(data.rpm));
		}

		break;
	}
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
		deactivate();
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
