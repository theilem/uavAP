/*
 * EmulationInterfaceHelper.h
 *
 *  Created on: Jan 22, 2018
 *      Author: mircot
 */

#ifndef SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_
#define SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_

#include "EmulationDirectInterface.h"

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/NetworkFactory.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>

#include "uavAP/API/AggregatableAutopilotAPI.h"

using EmulationInterfaceDefaults = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		DataPresentation,
		IDC,
		IPC,
		SignalHandler,
		AggregatableAutopilotAPI
		>;

using EmulationInterfaceHelper = StaticHelper<EmulationInterfaceDefaults,
		NetworkFactory,
		EmulationDirectInterface
		>;

#endif /* SRC_CORE_TOOLS_EMULATIONINTERFACE_EMULATIONINTERFACEHELPER_H_ */
