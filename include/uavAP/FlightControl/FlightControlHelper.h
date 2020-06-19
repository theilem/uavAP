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
