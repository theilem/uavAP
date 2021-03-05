/*
 * ConditionFactory.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_CONDITIONFACTORY_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_CONDITIONFACTORY_H_

#include <cpsCore/Framework/StaticFactory.h>
#include "uavAP/FlightAnalysis/Condition/DurationCondition.h"
#include "uavAP/FlightAnalysis/Condition/SteadyStateCondition.h"
#include "uavAP/FlightAnalysis/Condition/SensorDataCondition.h"

using ConditionFactory = StaticFactory<ICondition, false, DurationCondition, SteadyStateCondition, SensorDataCondition>;

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_CONDITIONFACTORY_H_ */
