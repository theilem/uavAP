/*
 * Relational.h
 *
 *  Created on: Mar 13, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RELATIONAL_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RELATIONAL_H_

#include "cpsCore/Utilities/EnumMap.hpp"

enum class Relational
{
	INVALID,
	EQUAL_TO,
	NOT_EQUAL_TO,
	GREATER_THAN,
	LESS_THAN,
	GREATER_THAN_EQUAL_TO,
	LESS_THAN_EQUAL_TO,
	NUM_RELATIONAL
};

ENUMMAP_INIT(Relational, { {Relational::EQUAL_TO, "="}, {Relational::NOT_EQUAL_TO, "!="},
		{Relational::GREATER_THAN, ">"}, {Relational::LESS_THAN, "<"},
		{Relational::GREATER_THAN_EQUAL_TO, ">="}, {Relational::LESS_THAN_EQUAL_TO, "<="}});

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RELATIONAL_H_ */
