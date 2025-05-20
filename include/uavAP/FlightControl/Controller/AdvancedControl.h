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

#include <cpsCore/Configuration/PropertyMapper.hpp>
#include <cpsCore/Configuration/Configuration.hpp>

#include <cpsCore/Utilities/EnumMap.hpp>

/**
 * Camber Control
 */
enum class CamberControl
{
	INVALID, NORMAL, THERMAL, CRUISE, NUM_CAMBER_CONTROL
};

ENUMMAP_INIT(CamberControl, {{CamberControl::NORMAL, "normal"}, {CamberControl::THERMAL, "thermal"},
		{CamberControl::CRUISE, "cruise"}});

/**
 * Special Control
 */
enum class SpecialControl
{
	INVALID, NONE, FLAP, CROW, NUM_SPECIAL_CONTROL
};

ENUMMAP_INIT(SpecialControl, {{SpecialControl::NONE, "none"}, {SpecialControl::FLAP, "flap"},
		{SpecialControl::CROW, "crow"}});

/**
 * Throws Control
 */
enum class ThrowsControl
{
	INVALID, NORMAL, CRUISE, NUM_THROWS_CONTROL
};

ENUMMAP_INIT(ThrowsControl, {{ThrowsControl::NORMAL, "normal"}, {ThrowsControl::CRUISE, "cruise"}});

/**
 * Advanced Controls
 */
enum class AdvancedControls
{
	INVALID, CAMBER_CONTROL, SPECIAL_CONTROL, THROWS_CONTROL, NUM_ADVANCED_CONTROLS
};

ENUMMAP_INIT(AdvancedControls, {{AdvancedControls::CAMBER_CONTROL, "camber_control"},
		{AdvancedControls::SPECIAL_CONTROL, "special_control"}, {AdvancedControls::THROWS_CONTROL,
		"throws_control"}});

/**
 * Advanced Control
 */
struct AdvancedControl
{
	Parameter<ThrowsControl> throwsSelection = {ThrowsControl::NORMAL, "throws_control", true};
	Parameter<CamberControl> camberSelection = {CamberControl::NORMAL, "camber_control", true};
	Parameter<SpecialControl> specialSelection = {SpecialControl::NONE, "special_control", true};
	Parameter<double> camberValue = {0.0, "camber_value", true};
	Parameter<double> specialValue = {0.0, "special_value", true};

	template<typename Configurator>
	void
	configure(Configurator& c)
	{
		c & throwsSelection;
		c & camberSelection;
		c & specialSelection;
		c & camberValue;
		c & specialValue;
	}
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADVANCEDCONTROL_H_ */
