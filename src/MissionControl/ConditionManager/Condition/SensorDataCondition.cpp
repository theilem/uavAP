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

#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SensorDataCondition.h"

SensorDataCondition::SensorDataCondition() :
		connection_(), trigger_()
{
}

//
//bool
//SensorDataCondition::configure(const Configuration& config)
//{
//	PropertyMapper<Configuration> pm(config);
//	Configuration dataFilterTree;
//
//	config_ = config;
//
//	if (!pm.add<bool>("filter_data", filterData_, false))
//	{
//		filterData_ = false;
//	}
//
//	if (!pm.add<bool>("use_tolerance", params.useTolerance(), false))
//	{
//		params.useTolerance() = false;
//	}
//
//	pm.addEnum<Sensor>("sensor", sensor_, true);
//	pm.addEnum<Relational>("relational", relational_, true);
//
//	if (params.useTolerance() && !pm.add<double>("tolerance", tolerance_, false))
//	{
//		params.useTolerance() = false;
//	}
//
//	if (filterData_ && pm.add("data_filter", dataFilterTree, false))
//	{
//		dataFilter_ = dataFilterFactory_.create(dataFilterTree);
//	}
//	else
//	{
//		filterData_ = false;
//	}
//
//	dataFilterInitialized_ = false;
//
//	return pm.map();
//}

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

	switch (params.sensor())
	{
	case SensorEnum::POSITION_X:
	{
		double filteredData = filterSensorData(data.position.x());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::POSITION_Y:
	{
		double filteredData = filterSensorData(data.position.y());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::POSITION_Z:
	{
		double filteredData = filterSensorData(data.position.z());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::VELOCITY_X:
	{
		double filteredData = filterSensorData(data.velocity.x());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::VELOCITY_Y:
	{
		double filteredData = filterSensorData(data.velocity.y());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::VELOCITY_Z:
	{
		double filteredData = filterSensorData(data.velocity.z());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::ACCELERATION_X:
	{
		double filteredData = filterSensorData(data.acceleration.x());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::ACCELERATION_Y:
	{
		double filteredData = filterSensorData(data.acceleration.y());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::ACCELERATION_Z:
	{
		double filteredData = filterSensorData(data.acceleration.z());

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::ATTITUDE_X:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.attitude.x()));

		if (params.useTolerance())
		{

			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case SensorEnum::ATTITUDE_Y:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.attitude.y()));

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case SensorEnum::ATTITUDE_Z:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.attitude.z()));

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case SensorEnum::ANGULAR_RATE_X:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.angularRate.x()));

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case SensorEnum::ANGULAR_RATE_Y:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.angularRate.y()));

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case SensorEnum::ANGULAR_RATE_Z:
	{
		double filteredDataDegrees = radToDeg(filterSensorData(data.angularRate.z()));

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredDataDegrees);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredDataDegrees);
		}

		break;
	}
	case SensorEnum::TIMESTAMP:
	{
		CPSLOG_ERROR << "SensorDataCondition: Timestamp Cannot Be Evaluated.";
		break;
	}
	case SensorEnum::HAS_GPS_FIX:
	{
		evaluation = evaluateSensorData<bool>(data.hasGPSFix);
		break;
	}
	case SensorEnum::AUTOPILOT_ACTIVE:
	{
		evaluation = evaluateSensorData<bool>(data.autopilotActive);
		break;
	}
	case SensorEnum::AIR_SPEED:
	{
		double filteredData = filterSensorData(data.airSpeed);

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::GROUND_SPEED:
	{
		double filteredData = filterSensorData(data.groundSpeed);

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::ANGLE_OF_ATTACK:
	{
		double filteredData = filterSensorData(data.angleOfAttack);

		if (params.useTolerance())
		{
			evaluation = evaluateSensorDataWithTolerance<double>(filteredData);
		}
		else
		{
			evaluation = evaluateSensorData<double>(filteredData);
		}

		break;
	}
	case SensorEnum::INVALID:
	{
		CPSLOG_ERROR << "SensorDataCondition: Invalid Sensor ";
		break;
	}
	default:
	{
		CPSLOG_ERROR << "SensorDataCondition: Unknown Sensor ";
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

//	if (filterData_)
//	{
//		if (!dataFilterInitialized_)
//		{
//			dataFilter_->initialize(sensorData);
//			dataFilterInitialized_ = true;
//		}
//
//		dataFilter_->filterData(sensorData);
//		sensorDataFiltered = dataFilter_->getFilteredData();
//	}

	return sensorDataFiltered;
}
