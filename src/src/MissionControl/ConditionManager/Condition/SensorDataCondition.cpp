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

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SensorDataCondition.h"

SensorDataCondition::SensorDataCondition() :
		config_(), connection_(), dataFilter_(), dataFilterFactory_(), trigger_(), sensor_(), relational_(), tolerance_(
				0), useTolerance_(false), filterData_(false), dataFilterInitialized_(false)
{
}

std::shared_ptr<SensorDataCondition>
SensorDataCondition::create(const Configuration& config)
{
	auto sensorDataCondition = std::make_shared<SensorDataCondition>();

	if (!sensorDataCondition->configure(config))
	{
		APLOG_ERROR << "SensorDataCondition: Failed to Load Config.";
	}

	return sensorDataCondition;
}

bool
SensorDataCondition::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	Configuration dataFilterTree;

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
		double filteredData = filterSensorData(data.position.x());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::POSITION_Y:
	{
		double filteredData = filterSensorData(data.position.y());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::POSITION_Z:
	{
		double filteredData = filterSensorData(data.position.z());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::VELOCITY_X:
	{
		double filteredData = filterSensorData(data.velocity.x());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::VELOCITY_Y:
	{
		double filteredData = filterSensorData(data.velocity.y());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::VELOCITY_Z:
	{
		double filteredData = filterSensorData(data.velocity.z());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::ACCELERATION_X:
	{
		double filteredData = filterSensorData(data.acceleration.x());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::ACCELERATION_Y:
	{
		double filteredData = filterSensorData(data.acceleration.y());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::ACCELERATION_Z:
	{
		double filteredData = filterSensorData(data.acceleration.z());

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::ATTITUDE_X:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.attitude.x()));

		if (useTolerance_)
		{

			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case Sensor::ATTITUDE_Y:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.attitude.y()));

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case Sensor::ATTITUDE_Z:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.attitude.z()));

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case Sensor::ANGULAR_RATE_X:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.angularRate.x()));

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case Sensor::ANGULAR_RATE_Y:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.angularRate.y()));

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case Sensor::ANGULAR_RATE_Z:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.angularRate.z()));

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
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
		double filteredData = filterSensorData(data.airSpeed);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::GROUND_SPEED:
	{
		double filteredData = filterSensorData(data.groundSpeed);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::ANGLE_OF_ATTACK:
	{
		double filteredData = filterSensorData(data.angleOfAttack);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::PROPULSION_POWER:
	{
		double filteredData = filterSensorData(data.propulsionPower);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::CONSUMED_ENERGY:
	{
		double filteredData = filterSensorData(data.consumedEnergy);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::BATTERY_VOLTAGE:
	{
		double filteredData = filterSensorData(data.batteryVoltage);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::BATTERY_CURRENT:
	{
		double filteredData = filterSensorData(data.batteryCurrent);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::THROTTLE:
	{
		double filteredData = filterSensorData(data.throttle);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case Sensor::RPM:
	{
		double filteredData = filterSensorData(data.rpm);

		if (useTolerance_)
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
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
