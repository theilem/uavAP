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
//#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/GlobalPlanner/GlobalPlannerFactory.h"
#include "uavAP/MissionControl/MissionPlanner/MissionPlannerFactory.h"
//#include "uavAP/MissionControl/Geofencing/Geofencing.h"
//#include "uavAP/MissionControl/Geofencing/GeofencingModelFactory.h"
#include "uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <uavAP/FlightControl/LocalPlanner/ForwardingLocalPlanner/ForwardingLocalPlanner.h>
//#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h"
//#include "uavAP/MissionControl/WindAnalysis/WindAnalysis.h"

using MissionControlDefaults = StaticHelper<IPC, SchedulerFactory, TimeProviderFactory, DataPresentation, SignalHandler,
                                            ForwardingLocalPlanner>;

using MissionControlHelper = StaticHelper<MissionControlDefaults,
                                          //		ConditionManager,
                                          //		ManeuverPlanner,
                                          //		GeofencingModelFactory,
                                          //		Geofencing,
                                          //		WindAnalysis,
                                          DataHandling,
                                          LocalFrameManager,
                                          MissionPlannerFactory,
                                          GlobalPlannerFactory
>;

#endif /* UAVAP_MISSIONCONTROL_MISSIONCONTROLHELPER_H_ */
