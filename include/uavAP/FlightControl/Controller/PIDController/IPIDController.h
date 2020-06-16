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
 * IPIDController.h
 *
 *  Created on: Jul 23, 2018
 *      Author: simonyu
 */

#ifndef CONTROLLER_IPIDCONTROLLER_H_
#define CONTROLLER_IPIDCONTROLLER_H_

#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControlElements/Control.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include <cpsCore/Configuration/Configuration.hpp>

class IPIDController : public IController
{
public:
	virtual
	~IPIDController() = default;

	virtual bool
	configure(const Configuration& config) = 0;

	virtual ControllerOutput
	getControllerOutput() = 0;

//	virtual std::shared_ptr<IPIDCascade>
//	getCascade() = 0;
};

#endif /* CONTROLLER_IPIDCONTROLLER_H_ */
