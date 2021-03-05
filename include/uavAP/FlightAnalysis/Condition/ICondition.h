/*
 * ICondition.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_ICONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_ICONDITION_H_

class ICondition
{

public:

	static constexpr auto typeId = "condition";

	virtual
	~ICondition() = default;

	virtual void
	initialize() = 0;

	virtual bool
	evaluate() = 0;

	virtual void
	printInfo()
	{ CPSLOG_ERROR << "No Info Printing implemented"; }
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_ICONDITION_H_ */
