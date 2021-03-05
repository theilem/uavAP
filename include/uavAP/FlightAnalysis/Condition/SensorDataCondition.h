/*
 * SensorDataCondition.h
 *
 *  Created on: Feb 26, 2021
 *      Author: Mirco Theile
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightAnalysis/Condition/Relational.h"
#include "uavAP/FlightAnalysis/Condition/ICondition.h"

class ISensingIO;

struct SensorDataConditionParams
{
	Parameter<bool> useTolerance = {false, "use_tolerance", false};
	Parameter<SensorEnum> sensor = {SensorEnum::INVALID, "sensor", true};
	Parameter<Relational> relational = {Relational::INVALID, "relational", true};
	Parameter<FloatingType> tolerance = {0.0, "tolerance", false};
	Parameter<FloatingType> threshold = {0.0, "threshold", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & useTolerance;
		c & sensor;
		c & relational;
		c & tolerance;
		c & threshold;
	}
};


class SensorDataCondition
		: public AggregatableObject<ISensingIO>,
		  public ConfigurableObject<SensorDataConditionParams>,
		  public ICondition
{
public:

	static constexpr TypeId typeId = "sensor_data";

	SensorDataCondition() = default;

	void
	initialize() override;

	bool
	evaluate() override;

};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_SENSORDATACONDITION_H_ */
