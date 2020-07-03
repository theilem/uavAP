/**
 *  @file         CommunicationHelper.h
 *  @author Simon Yu
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Helper Header File
 *
 *  Description
 */

#ifndef UAVAP_COMMUNICATION_COMMUNICATIONHELPER_H_
#define UAVAP_COMMUNICATION_COMMUNICATIONHELPER_H_

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/NetworkFactory.h>
#include <cpsCore/Utilities/IDC/TransportLayer/ITransportLayer.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>
#include "uavAP/Communication/Comm/CommFactory.h"

using CommunicationDefaults = StaticHelper<
		SignalHandler,
		IPC,
		IDC,
		SchedulerFactory,
		TimeProviderFactory,
		DataPresentation
		>;

using CommunicationHelper = StaticHelper<CommunicationDefaults,
		CommFactory,
		NetworkFactory
		>;
#endif /* UAVAP_COMMUNICATION_COMMUNICATIONHELPER_H_ */
