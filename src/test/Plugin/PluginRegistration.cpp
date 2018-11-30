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
/**
 * @file PluginRegistration.cpp
 * @date Sep 24, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#include <uavAP/FlightControl/Controller/ControllerFactory.h>
#include <uavAP/FlightControl/LocalPlanner/ILocalPlanner.h>

#include "TestControllerPlugin.h"
#include <uavAP/Core/Framework/PluginAPI.h>

extern "C"
{
void
register_plugin()
{
	Factory<IController>::registerExternalCreator<TestControllerPlugin>();
//	Factory<ILocalPlanner>::registerExternalCreator<TestControllerPlugin>();
}
}

