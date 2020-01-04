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
/*
 * FlightControlHelper.h
 *
 *  Created on: Jul 26, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_
#define UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_

#include <uavAP/Core/IDC/NetworkFactory.h>
#include "uavAP/API/ChannelMixing.h"
#include "uavAP/API/Simulation/Connector/SimulationConnector.h"
#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/Framework/Helper.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"

class SimulationHelper: public Helper
{
public:
	SimulationHelper()
	{
		addCreator<SimulationConnector>();
		addCreator<ChannelMixing>();

		addDefault<SchedulerFactory>();
		addDefault<TimeProviderFactory>();
		addDefaultCreator<IPC>();
		addDefault<NetworkFactory>();
		addDefault<DataPresentation>();
	}
};

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_ */
