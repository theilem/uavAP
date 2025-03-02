/*
 * ApproachPlanner.h
 *
 *  Created on: Dec 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_APPROACHPLANNER_APPROACHPLANNER_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_APPROACHPLANNER_APPROACHPLANNER_H_

#include <cpsCore/cps_object>

#include <cpsCore/Utilities/IPC/Publisher.h>
#include <uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlannerParams.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"

class ILocalPlanner;

class IPC;

class DataPresentation;
class DataHandling;

class ApproachPlanner : public IGlobalPlanner,
							public AggregatableObject<ILocalPlanner, DataHandling>,
							public ConfigurableObject<ApproachPlannerParams>,
							public IRunnableObject
{
public:

	static constexpr TypeId typeId = "spline";

	ApproachPlanner() = default;

	bool
	run(RunStage stage) override;

	void
	setMission(const Mission& mission) override;

	Mission
	getMission() const override;

private:

	PathSections
	createNaturalSplines(const Mission& mission);

	PathSections
	createCatmulRomSplines(const Mission& mission);

	Optional<Mission>
	missionRequest(const DataRequest& request);

	Mission mission_;
};

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_APPROACHPLANNER_APPROACHPLANNER_H_ */
