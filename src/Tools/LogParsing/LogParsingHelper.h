//
// Created by mirco on 24.06.20.
//

#ifndef UAVAP_LOGPARSINGHELPER_H
#define UAVAP_LOGPARSINGHELPER_H

#include <cpsCore/Framework/StaticHelper.h>
#include "LogParser.h"
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>

using LogParsingHelper = StaticHelper<LogParser, SchedulerFactory, TimeProviderFactory>;


#endif //UAVAP_LOGPARSINGHELPER_H
