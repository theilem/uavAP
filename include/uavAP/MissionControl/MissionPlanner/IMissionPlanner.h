/*
 * IMissionPlanner.h
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_IMISSIONPLANNER_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_IMISSIONPLANNER_H_

class IMissionPlanner
{
public:

	static constexpr const char* const typeId = "mission_planner";

	virtual
	~IMissionPlanner() = default;

	virtual void
	missionRequest(const std::string& mission) = 0;
};

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_IMISSIONPLANNER_H_ */
