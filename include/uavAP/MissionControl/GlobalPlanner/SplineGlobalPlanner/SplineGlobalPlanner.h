/*
 * SplineGlobalPlanner.h
 *
 *  Created on: Dec 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_SPLINEGLOBALPLANNER_SPLINEGLOBALPLANNER_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_SPLINEGLOBALPLANNER_SPLINEGLOBALPLANNER_H_

#include <cpsCore/cps_object>

#include <cpsCore/Utilities/IPC/Publisher.h>
#include <uavAP/MissionControl/GlobalPlanner/SplineGlobalPlanner/SplineGlobalPlannerParams.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"

class ILocalPlanner;

class IPC;

class DataPresentation;
class DataHandling;

class SplineGlobalPlanner : public IGlobalPlanner,
							public AggregatableObject<ILocalPlanner, DataHandling>,
							public ConfigurableObject<SplineGlobalPlannerParams>,
							public IRunnableObject
{
public:

	static constexpr TypeId typeId = "spline";

	SplineGlobalPlanner() = default;

	bool
	run(RunStage stage) override;

	void
	setMission(const Mission& mission) override;

	Mission
	getMission() const override;

private:

	Trajectory
	createNaturalSplines(const Mission& mission);

	Trajectory
	createCatmulRomSplines(const Mission& mission);

	Optional<Mission>
	missionRequest(const DataRequest& request);

	Mission mission_;
};

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_SPLINEGLOBALPLANNER_SPLINEGLOBALPLANNER_H_ */
