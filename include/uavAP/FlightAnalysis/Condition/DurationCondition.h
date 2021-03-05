/*
 * DurationCondition.h
 *
 *  Created on: Feb 26, 2021
 *      Author: Mirco Theile
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_DURATIONCONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_DURATIONCONDITION_H_

#include <cpsCore/cps_object>
#include "uavAP/FlightAnalysis/Condition/ICondition.h"

class ITimeProvider;

struct DurationConditionParams
{
	Parameter<int> duration = {0, "duration", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & duration;
	}
};

class DurationCondition
		: public ICondition,
		  public AggregatableObject<ITimeProvider>,
		  public ConfigurableObject<DurationConditionParams>
{
public:

	static constexpr TypeId typeId = "duration";

	DurationCondition() = default;

	void
	initialize() override;

	bool
	evaluate() override;

	void
	printInfo() override;


private:

	TimePoint init_;
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_DURATIONCONDITION_H_ */
