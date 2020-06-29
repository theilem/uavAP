//
// Created by mirco on 24.06.20.
//

#ifndef UAVAP_LOGPARSINGHELPER_H
#define UAVAP_LOGPARSINGHELPER_H

#include <cpsCore/Framework/StaticHelper.h>
#include "LogParser.h"
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>

using LogParsingDefaults = StaticHelper<SchedulerFactory, TimeProviderFactory, SignalHandler>;
using LogParsingHelper = StaticHelper<LogParsingDefaults, LogParser>;


#endif //UAVAP_LOGPARSINGHELPER_H
