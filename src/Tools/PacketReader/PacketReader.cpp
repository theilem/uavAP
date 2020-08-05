//
// Created by mirco on 05.08.20.
//

#include <cpsCore/Configuration/TerminalConfigurator.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/Serial/SerialHandlerParams.h>
#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/Serial/SerialNetworkLayer.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/Scheduler/MultiThreadingScheduler.h>
#include <cpsCore/Utilities/TimeProvider/SystemTimeProvider.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>
#include <cpsCore/Synchronization/SimpleRunner.h>

#include "uavAP/Core/DataHandling/Content.hpp"


void
onPacket(const Packet& p, const std::shared_ptr<DataPresentation>& dp)
{
	std::cout << "Packet received, Content: " << (int)dp->getHeader<Content>(p) << std::endl;
}

int
main()
{
	SerialHandlerParams serialParams;
	TerminalConfigurator tconf;

	tconf & serialParams;

	auto agg = StaticHelper<SystemTimeProvider,
			SignalHandler,
			MultiThreadingScheduler,
			IDC,
			DataPresentation,
			SerialNetworkLayer>::createDefaultAggregation(Configuration());
	auto snl = agg.getOne<SerialNetworkLayer>();
	auto idc = agg.getOne<IDC>();
	auto sched = agg.getOne<IScheduler>();
	auto dp = agg.getOne<DataPresentation>();

	sched->setMainThread();

	SerialNetworkLayerParams params;
	params.ports().clear();
	params.ports().insert(std::make_pair("reader", serialParams));
	snl->setParams(params);

	SimpleRunner runner(agg);

	if (runner.runAllStages())
	{
		return 1;
	}
	idc->subscribeOnPacket("reader", [dp](const auto& p){onPacket(p, dp);});

	sched->startSchedule();

	return 0;


}