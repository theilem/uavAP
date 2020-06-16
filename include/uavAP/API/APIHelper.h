/*
 * APIHelper.h
 *
 *  Created on: Jul 12, 2018
 *      Author: mircot
 */

#ifndef UAVAP_API_APIHELPER_H_
#define UAVAP_API_APIHELPER_H_


#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>

using APIHelper = StaticHelper<TimeProviderFactory, SchedulerFactory, IPC, DataPresentation, SignalHandler>;

#endif /* UAVAP_API_APIHELPER_H_ */
