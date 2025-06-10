//
// Created by Mirco Theile on 29/5/25.
//


#include "cpsCore/Configuration/JsonPopulator.h"
#include "cpsCore/Framework/StaticHelper.h"
#include "cpsCore/Synchronization/SynchronizedRunner.h"
#include "cpsCore/Utilities/IDC/NetworkLayer/NetworkFactory.h"
#include "uavAP/ExternalPlanner/ExtConnector.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingIO.h"
#include "cpsCore/Utilities/IDC/IDC.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "cpsCore/Utilities/DataPresentation/DataPresentation.h"
#include "cpsCore/Utilities/Scheduler/SchedulerFactory.h"
#include "cpsCore/Utilities/TimeProvider/TimeProviderFactory.h"
#include "cpsCore/Utilities/SignalHandler/SignalHandler.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

int
main(int argc, char** argv)
{
    using ExternalPlannerHelper = StaticHelper<TimeProviderFactory, SchedulerFactory, ExtConnector, IPC, IDC, NetworkFactory,
                                               DataPresentation, SensingIO, DataHandling<Content, Target>>;
    if (argc < 2)
    {
        auto pop = JsonPopulator::populateContainer<ExternalPlannerHelper>();
        std::cout << pop.getString() << std::endl;
        return 0;
    }
    std::string configPath = argv[1];
    auto aggregator = ExternalPlannerHelper::createAggregation(configPath);
    auto sched = aggregator.getOne<IScheduler>();
    sched->setMainThread();

    CPSLOG_DEBUG << "Run synchronized";
    SynchronizedRunner runner;
    if (runner.runSynchronized(aggregator))
    {
        CPSLOG_ERROR << "Something went wrong";
        return 1;
    }

    sched->startSchedule();

    //Terminated -> Cleanup
    aggregator.cleanUp();

    return 0;
}
