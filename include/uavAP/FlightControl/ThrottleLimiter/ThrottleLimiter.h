//
// Created by mirco on 03.08.20.
//

#ifndef UAVAP_THROTTLELIMITER_H
#define UAVAP_THROTTLELIMITER_H


#include <cpsCore/cps_object>

#include "uavAP/FlightControl/ThrottleLimiter/ThrottleLimiterParams.h"

class ManeuverRatePIDController;
class DataHandling;

class ThrottleLimiter
		: public AggregatableObject<ManeuverRatePIDController, DataHandling>,
		  public IRunnableObject,
		  public ConfigurableObject<ThrottleLimiterParams>
{
public:

	static constexpr TypeId typeId = "throttle_limiter";

	bool
	run(RunStage stage) override;

private:

	void
	applyThrottleLimit();


};

#endif //UAVAP_THROTTLELIMITER_H
