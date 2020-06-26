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
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/FlightControl/LocalPlanner/LocalPlannerFactory.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIOFactory.h"

#include <cpsCore/Framework/StaticHelper.h>

//class FlightControlHelper : public Helper
//{
//public:
//	FlightControlHelper()
//	{
//		addFactory<LocalPlannerFactory>();
//		addFactory<ControllerFactory>();
//
//		addDefault<SchedulerFactory>();
//		addDefault<TimeProviderFactory>();
//		addDefaultConfigurable<IPC>();
//		addDefault<SensingActuationIOFactory>();
//		addConfigurable<DataHandling>();
//		addDefaultCreator<DataPresentation>();
//	}
//};

using FlightControlHelper = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		IPC,
		DataPresentation,
		SensingActuationIOFactory,
		DataHandling,
		ControllerFactory,
		LocalPlannerFactory,
		SignalHandler>;

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_ */
