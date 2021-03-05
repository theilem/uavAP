//
// Created by mirco on 25.02.21.
//

#ifndef UAVAP_OVERRIDESAFETY_H
#define UAVAP_OVERRIDESAFETY_H

#include <cpsCore/cps_object>
#include "uavAP/FlightControl/Safety/OverrideSafetyParams.h"

class IScheduler;

class ISensingIO;

class OverrideHandler;
class DataHandling;

class OverrideSafety
		: public AggregatableObject<IScheduler, ISensingIO, OverrideHandler, DataHandling>,
		  public ConfigurableObject<OverrideSafetyParams>,
		  public IRunnableObject
{
public:

	static constexpr TypeId typeId = "override_safety";

	bool
	run(RunStage stage) override;

private:

	void
	checkRectanguloid();


};


#endif //UAVAP_OVERRIDESAFETY_H
