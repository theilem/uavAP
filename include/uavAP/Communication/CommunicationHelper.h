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

#include "uavAP/Core/IDC/NetworkLayer/NetworkFactory.h"
#include "uavAP/Core/IDC/IDC.h"
#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/Framework/Helper.h"

#include "uavAP/Communication/Comm/CommFactory.h"

class CommunicationHelper: public Helper
{
public:

	CommunicationHelper()
	{
		addDefaultConfigurable<IPC>();
		addDefaultCreator<IDC>();
		addDefault<SchedulerFactory>();
		addDefault<TimeProviderFactory>();
		addDefaultCreator<DataPresentation>();

		addFactory<CommFactory>();
		addFactory<NetworkFactory>();
	}

};

#endif /* UAVAP_COMMUNICATION_COMMUNICATIONHELPER_H_ */
