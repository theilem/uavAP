//
// Created by mirco on 26.02.21.
//

#ifndef UAVAP_MANEUVERPLANNER_H
#define UAVAP_MANEUVERPLANNER_H

#include <cpsCore/cps_object>
#include <cpsCore/Aggregation/AggregatableObjectMaster.hpp>
#include <cpsCore/Utilities/IPC/Publisher.h>
#include "uavAP/FlightAnalysis/ManeuverPlanner/ManeuverPlannerParams.h"

class DataHandling;
class IScheduler;
class IPC;

class ManeuverPlanner
		: public AggregatableObjectMaster<DataHandling, IScheduler, IPC>,
		  public ConfigurableObject<ManeuverPlannerParams>,
		  public IRunnableObject
{
public:

	static constexpr TypeId typeId = "maneuver_planner";

	bool
	run(RunStage stage) override;

private:

	void
	maneuverSelection(const std::string& maneuverId);

	void
	startManeuverSet();

	std::shared_ptr<Maneuver>
	createManeuver(const ManeuverParams& p);

	void
	activateManeuver();

	void
	checkManeuver();

	void
	stopManeuver();

	Publisher<ManeuverOverride> publisher_;
	std::pair<const std::string, ManeuverSet>* activeManeuverSet_;
	ManeuverSet::iterator activeManeuver_;

	std::shared_ptr<Maneuver> maneuver_;
};


#endif //UAVAP_MANEUVERPLANNER_H
