////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
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


using CommunicationHelper = StaticHelper<
        SignalHandler,
		IPC,
		IDC,
		SchedulerFactory,
		TimeProviderFactory,
		DataPresentation,
		CommFactory,
		NetworkFactory>;
#endif /* UAVAP_COMMUNICATION_COMMUNICATIONHELPER_H_ */
