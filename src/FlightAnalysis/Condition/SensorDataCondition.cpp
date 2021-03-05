/*
 * SensorDataCondition.cpp
 *
 *  Created on: Feb 26, 2021
 *      Author: Mirco Theile
 */

#include "uavAP/FlightAnalysis/Condition/SensorDataCondition.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"

void
SensorDataCondition::initialize()
{
	if (!checkIsSetAll())
		CPSLOG_ERROR << "SensIO missing";
}

bool
SensorDataCondition::evaluate()
{
	auto sensIO = get<ISensingIO>();
	if (!sensIO)
	{
		CPSLOG_ERROR << "SensIO missing";
		return true;
	}
	auto value = enumAccess<FloatingType>(sensIO->getSensorData(), params.sensor());
	if (params.useTolerance())
		return evaluateRelationalTolerance(value, params.threshold(), params.tolerance(), params.relational());
	return evaluateRelational(value, params.threshold(), params.relational());
}
