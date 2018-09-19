////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * SpecialControl.h
 *
 *  Created on: Jul 12, 2018
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADVANCEDCONTROL_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADVANCEDCONTROL_H_
#include <map>

/**
 * Special Control
 */

enum class SpecialControl
{
	NONE, FLAP, CROW
};

const static std::map<SpecialControl, std::string> SpecialControlBimapLeft =
{
{ SpecialControl::NONE, "none" },
{ SpecialControl::FLAP, "flap" },
{ SpecialControl::CROW, "crow" } };

const static std::map<std::string, SpecialControl> SpecialControlBimapRight =
{
{ "none", SpecialControl::NONE },
{ "flap", SpecialControl::FLAP },
{ "crow", SpecialControl::CROW } };

/**
 * Camber Control
 */

enum class CamberControl
{
	NORMAL, THERMAL, CRUISE
};

const static std::map<CamberControl, std::string> CamberBimapLeft =
{
{ CamberControl::NORMAL, "normal" },
{ CamberControl::THERMAL, "thermal" },
{ CamberControl::CRUISE, "cruise" } };

const static std::map<std::string, CamberControl> CamberBimapRight =
{
{ "normal", CamberControl::NORMAL },
{ "thermal", CamberControl::THERMAL },
{ "cruise", CamberControl::CRUISE } };

/**
 * Throws Control
 */

enum class ThrowsControl
{
	NORMAL, CRUISE
};

const static std::map<ThrowsControl, std::string> ThrowsBimapLeft =
{
{ ThrowsControl::NORMAL, "normal" },
{ ThrowsControl::CRUISE, "cruise" } };

const static std::map<std::string, ThrowsControl> ThrowsBimapRight =
{
{ "normal", ThrowsControl::NORMAL },
{ "cruise", ThrowsControl::CRUISE } };

/**
 * Advanced Control
 */

struct AdvancedControl
{
	ThrowsControl throwsSelection = ThrowsControl::NORMAL;
	CamberControl camberSelection = CamberControl::NORMAL;
	SpecialControl specialSelection = SpecialControl::NONE;
	double camberValue = 0.0;
	double specialValue = 0.0;
};


namespace dp
{
template<class Archive, typename Type>
void
serialize(Archive& ar, AdvancedControl& t)
{
	ar & t.throwsSelection;
	ar & t.camberSelection;
	ar & t.specialSelection;
	ar & t.camberValue;
	ar & t.specialValue;
}
}



#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADVANCEDCONTROL_H_ */
