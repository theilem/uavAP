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
	EQUAL,
	NOT_EQUAL,
	GREATER,
	LESS,
	GREATER_EQUAL,
	LESS_EQUAL,
	NUM_RELATIONAL
};

ENUMMAP_INIT(Relational, {
	{ Relational::EQUAL, "=" },
	{ Relational::NOT_EQUAL, "!=" },
	{ Relational::GREATER, ">" },
	{ Relational::LESS, "<" },
	{ Relational::GREATER_EQUAL, ">=" },
	{ Relational::LESS_EQUAL, "<=" }
});

template <typename Type>
bool
evaluateRelational(const Type& a, const Type& b, Relational rel)
{
	switch (rel)
	{
		case Relational::INVALID:
			return false;
		case Relational::EQUAL:
			return a == b;
		case Relational::NOT_EQUAL:
			return a != b;
		case Relational::GREATER:
			return a > b;
		case Relational::LESS:
			return a < b;
		case Relational::GREATER_EQUAL:
			return a >= b;
		case Relational::LESS_EQUAL:
			return a <= b;
		case Relational::NUM_RELATIONAL:
			return false;
	}
	return false;
}

template <typename Type>
bool
evaluateRelationalTolerance(const Type& a, const Type& b, const Type& tolerance, Relational rel)
{
	switch (rel)
	{
		case Relational::INVALID:
			return false;
		case Relational::EQUAL:
			return a <= b + tolerance && a >= b - tolerance;
		case Relational::NOT_EQUAL:
			return a > b + tolerance || a < b - tolerance;
		case Relational::GREATER:
			return a > b + tolerance;
		case Relational::LESS:
			return a < b - tolerance;
		case Relational::GREATER_EQUAL:
			return a >= b + tolerance;
		case Relational::LESS_EQUAL:
			return a <= b - tolerance;
		case Relational::NUM_RELATIONAL:
			return false;
	}
	return false;
}


template <typename Type, Relational rel>
bool
evaluateRelationalTolerance(const Type& a, const Type& b, const Type& tolerance)
{
	switch (rel)
	{
		case Relational::INVALID:
			return false;
		case Relational::EQUAL:
			return a <= b + tolerance && a >= b - tolerance;
		case Relational::NOT_EQUAL:
			return a > b + tolerance || a < b - tolerance;
		case Relational::GREATER:
			return a > b + tolerance;
		case Relational::LESS:
			return a < b - tolerance;
		case Relational::GREATER_EQUAL:
			return a >= b + tolerance;
		case Relational::LESS_EQUAL:
			return a <= b - tolerance;
		case Relational::NUM_RELATIONAL:
			return false;
	}
	return false;
}

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_RELATIONAL_H_ */
