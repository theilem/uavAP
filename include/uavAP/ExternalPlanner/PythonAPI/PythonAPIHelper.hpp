//
// Created by mirco on 11.10.24.
//

#ifndef UAVAP_PYTHONAPIHELPER_HPP
#define UAVAP_PYTHONAPIHELPER_HPP

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include "uavAP/FlightControl/LocalPlanner/ForwardingLocalPlanner/ForwardingLocalPlanner.h"

using PythonAPIDefaults = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		IPC,
		DataPresentation,
		ForwardingLocalPlanner
>;

using PythonAPIHelper = StaticHelper<PythonAPIDefaults
>;

#endif //UAVAP_PYTHONAPIHELPER_HPP
