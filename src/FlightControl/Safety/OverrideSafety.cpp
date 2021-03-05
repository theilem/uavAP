//
// Created by mirco on 25.02.21.
//

#include "uavAP/FlightControl/Safety/OverrideSafety.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/Core/OverrideHandler/OverrideHandler.h"

#include <cpsCore/Utilities/Scheduler/IScheduler.h>

bool
OverrideSafety::run(RunStage stage)
{
	switch (stage)
	{

		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing deps";
				return true;
			}
			break;
		}
		case RunStage::NORMAL:
		{
			auto sched = get<IScheduler>();
			sched->schedule([this]()
							{ checkRectanguloid(); }, Milliseconds(params.period()), Milliseconds(params.period()));

			auto dh = get<DataHandling>();
			dh->addTriggeredStatusFunction<Rectanguloid, DataRequest>(
					[this](const DataRequest& req) -> Optional<Rectanguloid>
					{
						if (req == DataRequest::SAFETY_BOUNDS) return params.rectanguloid();
						return std::nullopt;
					}, Content::SAFETY_BOUNDS, Content::REQUEST_DATA);
			dh->addConfig(this, Content::OVERRIDE_SAFETY_PARAMS);
			break;
		}
		default:
			break;
	}
	return false;
}

void
OverrideSafety::checkRectanguloid()
{
	auto io = get<ISensingIO>();
	auto oh = get<OverrideHandler>();
	auto sd = io->getSensorData();

	if (!params.rectanguloid().isInside(sd.position))
		oh->disable();
	else
		oh->enable();

}
