//
// Created by mirco on 27.08.20.
//

#ifndef UAVAP_APEXTHELPER_H
#define UAVAP_APEXTHELPER_H

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/TimeProvider/SystemTimeProvider.h>
#include <cpsCore/Utilities/Scheduler/MultiThreadingScheduler.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>

#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/API/ap_ext/LinearSensorManager.h"
#include "uavAP/API/AggregatableAutopilotAPI.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

using ApExtHelperDefaults = StaticHelper<
        DataPresentation,
		IPC,
		SystemTimeProvider,
		MultiThreadingScheduler,
		SignalHandler,
		AggregatableAutopilotAPI
>;

using ApExtHelper = StaticHelper<ApExtHelperDefaults,
		ApExtManager,
		LinearSensorManager,
		DataHandling
>;

#endif //UAVAP_APEXTHELPER_H
