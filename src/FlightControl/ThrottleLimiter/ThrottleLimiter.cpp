//
// Created by mirco on 03.08.20.
//

#include "uavAP/FlightControl/ThrottleLimiter/ThrottleLimiter.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRatePIDController.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

bool
ThrottleLimiter::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing dependencies";
				return true;
			}
			break;
		}
		case RunStage::NORMAL:
		{
			auto dh = get<DataHandling>();

			dh->addConfig(this, Content::THROTTLE_LIMITER_PARAMS, [this]{applyThrottleLimit();});
			applyThrottleLimit();
			break;
		}
		default:
			break;
	}

	return false;

}

void
ThrottleLimiter::applyThrottleLimit()
{
	auto mc = get<ManeuverRatePIDController>();

	if (params.applyThrottleLimit())
		mc->setThrottleLimit(std::min(params.throttleLimit(), 1.0f));
	else
		mc->setThrottleLimit(1.0);
}
