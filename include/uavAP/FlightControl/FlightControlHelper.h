/*
 * FlightControlHelper.h
 *
 *  Created on: Jul 26, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_
#define UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_

#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>
#include "uavAP/FlightControl/Controller/ControllerFactory.h"
#include "uavAP/FlightControl/ThrottleLimiter/ThrottleLimiter.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/FlightControl/LocalPlanner/LocalPlannerFactory.h"
#include <uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h>
#include "uavAP/FlightControl/Safety/OverrideSafety.h"

#include <cpsCore/Framework/StaticHelper.h>
#include "uavAP/Core/OverrideHandler/OverrideHandler.h"

using FlightControlDefaults = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		IPC,
		DataPresentation,
		SignalHandler,
		SensingActuationIO
		>;

using FlightControlHelper = StaticHelper<FlightControlDefaults,
		DataHandling<Content, Target>,
		ControllerFactory,
		LocalPlannerFactory,
		ThrottleLimiter,
		OverrideHandler,
		OverrideSafety
		>;

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_ */
