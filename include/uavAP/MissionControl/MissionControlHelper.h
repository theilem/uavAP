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
 * MissionControlHelper.h
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONCONTROLHELPER_H_
#define UAVAP_MISSIONCONTROL_MISSIONCONTROLHELPER_H_

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
//#include "uavAP/MissionControl/DataHandling/MissionControlDataHandling.h"
//#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/GlobalPlanner/GlobalPlannerFactory.h"
#include "uavAP/MissionControl/MissionPlanner/MissionPlannerFactory.h"
//#include "uavAP/MissionControl/Geofencing/Geofencing.h"
//#include "uavAP/MissionControl/Geofencing/GeofencingModelFactory.h"
#include "uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>
//#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h"
//#include "uavAP/MissionControl/WindAnalysis/WindAnalysis.h"
//
//class MissionControlHelper: public Helper
//{
//public:
//	MissionControlHelper()
//	{
//		addDefaultConfigurable<IPC>();
//		addDefaultCreator<MissionControlDataHandling>();
//		addDefaultCreator<ConditionManager>();
//
//		addDefault<SchedulerFactory>();
//		addDefault<TimeProviderFactory>();
//		addDefaultCreator<DataPresentation>();
//
//		addCreator<ManeuverPlanner>();
//		addCreator<LocalFrameManager>();
//		addFactory<MissionPlannerFactory>();
//		addFactory<GlobalPlannerFactory>();
//		addFactory<GeofencingModelFactory>();
//		addConfigurable<Geofencing>();
//		addConfigurable<WindAnalysis>();
//	}
//};

//using MissionControlHelper = StaticHelper<
//		IPC,
//		MissionControlDataHandling,
//		ConditionManager,
//		SchedulerFactory,
//		TimeProviderFactory,
//		DataPresentation,
//		ManeuverPlanner,
//		LocalFrameManager,
//		MissionPlannerFactory,
//		GlobalPlannerFactory,
//		GeofencingModelFactory,
//		Geofencing,
//		WindAnalysis>;

using MissionControlHelper = StaticHelper<
		IPC,
//		MissionControlDataHandling,
//		ConditionManager,
		SchedulerFactory,
		TimeProviderFactory,
		DataPresentation,
		DataHandling,
//		ManeuverPlanner,
		LocalFrameManager,
		MissionPlannerFactory,
		GlobalPlannerFactory,
		SignalHandler
//		GeofencingModelFactory,
//		Geofencing,
//		WindAnalysis
		>;

#endif /* UAVAP_MISSIONCONTROL_MISSIONCONTROLHELPER_H_ */
