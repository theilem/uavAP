//
// Created by mirco on 26.02.21.
//

#ifndef UAVAP_MANEUVERPLANNER_H
#define UAVAP_MANEUVERPLANNER_H

#include <cpsCore/cps_object>
#include <cpsCore/Aggregation/AggregatableObjectMaster.hpp>
#include <cpsCore/Utilities/IPC/Publisher.h>
#include <fstream>

#include "uavAP/Core/DataHandling/Content.hpp"
#include "uavAP/FlightAnalysis/ManeuverPlanner/ManeuverPlannerParams.h"

template <typename C, typename T>
class DataHandling;
class IScheduler;
class IPC;
class ISensingIO;

class ManeuverPlanner
		: public AggregatableObjectMaster<DataHandling<Content, Target>, IScheduler, IPC, ISensingIO>,
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

	Publisher<Maneuver::Overrides> overridePublisher_;
	Publisher<Maneuver::Maintains> maintainsPublisher_;
	std::pair<const std::string, ManeuverSet>* activeManeuverSet_;
	ManeuverSet::iterator activeManeuver_;

	std::shared_ptr<Maneuver> maneuver_;
	std::ofstream maneuverLogFile_;
};


#endif //UAVAP_MANEUVERPLANNER_H
