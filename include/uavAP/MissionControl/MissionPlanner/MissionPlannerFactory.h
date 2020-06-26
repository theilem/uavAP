/*
 * MissionPlannerFactory.h
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_MISSIONPLANNERFACTORY_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_MISSIONPLANNERFACTORY_H_

#include <cpsCore/Framework/StaticFactory.h>

#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/CustomPlanner/CustomPlanner.h"
//#include "uavAP/MissionControl/MissionPlanner/SimpleMissionPlanner/SimpleMissionPlanner.h"

//class MissionPlannerFactory: public Factory<IMissionPlanner>
//{
//public:
//	MissionPlannerFactory()
//	{
//		addCreator<SimpleMissionPlanner>();
//		addCreator<CustomPlanner>();
//	}
//
//};

using MissionPlannerFactory = StaticFactory<IMissionPlanner, false, CustomPlanner>;

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MISSIONPLANNERFACTORY_H_ */
