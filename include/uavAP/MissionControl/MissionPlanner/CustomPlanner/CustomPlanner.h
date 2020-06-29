/*
 * ManeuverPlanner.h
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_CUSTOMPLANNER_H_

#include <memory>
#include <map>

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <uavAP/Core/DataHandling/Content.hpp>

#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"
#include "uavAP/MissionControl/MissionPlanner/CustomPlanner/CustomPlannerParams.h"

class IPC;

class IGlobalPlanner;

class IScheduler;

class DataHandling;

struct SensorData;
enum class DataRequest;

/**
 * @brief   The CustomPlanner class is a mission planner that can accept
 *          maneuver overrides from the ground station. It's controller cascade
 *          has switches which can be switched from PID output to override output.
 */
class CustomPlanner
		: public IMissionPlanner,
		  public AggregatableObject<IPC, IGlobalPlanner, IScheduler, DataHandling>,
		  public IRunnableObject,
		  public ConfigurableObject<CustomPlannerParams>
{
public:

	static constexpr TypeId typeId = "custom";

	CustomPlanner();

	bool
	run(RunStage stage) override;

	/**
	 * @brief   missionRequest searches for a list of waypoints with a name
	 *          matching mission. If found, it sets the current mission to that
	 *          set of waypoints and then begins to fly it.
	 * @param   mission name of set of waypoints to fly
	 */
	void
	missionRequest(const std::string& mission) override;

private:

	using MissionMap = std::map<std::string, Mission>;

	/**
	 * @brief   publishMission notifies the global planner of the new mission to
	 *          fly
	 */
	void
	publishMission();

	/**
	 * @brief   onSensorData is called every time sensor data is received. This
	 *          function checks to see if the aircraft has left safety bounds
	 *          and if so, disables the override
	 * @param   data sensor data containing current override position
	 */
	void
	onSensorData(const SensorData& data);


	double defaultVelocity_;
	double defaultAltitude_;
	bool useApproach_;

	Subscription sensorDataSubscription_;

	MissionMap::const_iterator currentMission_;

	Vector3 currentPosition_;
	Vector3 currentDirection_;
	std::mutex positionMutex_;
};

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_H_ */
