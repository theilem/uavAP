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

#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/Core/DataPresentation/DataPresentationFactory.h"
#include "uavAP/Core/Framework/Helper.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/SchedulerFactory.h"
#include "uavAP/Core/TimeProvider/TimeProviderFactory.h"
#include "uavAP/FlightControl/Controller/ControllerFactory.h"
#include "uavAP/FlightControl/DataHandling/FlightControlDataHandling.h"
#include "uavAP/FlightControl/LocalPlanner/LocalPlannerFactory.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIOFactory.h"

class FlightControlHelper: public Helper
{
public:
	FlightControlHelper()
	{
		addFactory<LocalPlannerFactory>("local_planner");
		addFactory<ControllerFactory>("controller");

		addDefault<SchedulerFactory>("scheduler");
		addDefault<TimeProviderFactory>("time_provider");
		addDefaultCreator<IPC>("ipc");
		addDefault<SensingActuationIOFactory>("sens_act_io");
		addDefaultCreator<FlightControlDataHandling>("data_handling");
		addDefault<DataPresentationFactory<Content,Target>>("data_presentation");
	}
};

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLHELPER_H_ */
