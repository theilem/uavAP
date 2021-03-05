/*
 * SteadyStateCondition.h
 *
 *  Created on: Feb 26, 2021
 *      Author: Mirco Theile
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_STEADYSTATECONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_STEADYSTATECONDITION_H_

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/FlightAnalysis/Condition/ICondition.h"
#include "uavAP/Core/SensorData.h"

class ISensingIO;

class ITimeProvider;

struct SteadyStateConditionParams
{
	Parameter<int> duration = {500, "duration", true};
	Parameter<std::map<SensorEnum, FloatingType>> sensorValues = {{}, "sensor_values", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & duration;
		c & sensorValues;
	}
};

class SteadyStateCondition
		: public ICondition,
		  public ConfigurableObject<SteadyStateConditionParams>,
		  public AggregatableObject<ISensingIO, ITimeProvider>
{
public:

	static constexpr const char* const typeId = "steady_state";

	SteadyStateCondition() = default;

	void
	initialize() override;

	bool
	evaluate() override;

private:

	std::map<SensorEnum, FloatingType> lastReadings_;
	TimePoint firstSteadyState_;

};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_STEADYSTATECONDITION_H_ */
