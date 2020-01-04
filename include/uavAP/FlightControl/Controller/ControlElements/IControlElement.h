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
 * ControlElements.h
 *
 *  Created on: Jun 15, 2017
 *      Author: mircot
 */

#ifndef FLIGHTCONTROLLER_CONTROLELEMENTS_CONTROLELEMENTS_H_
#define FLIGHTCONTROLLER_CONTROLELEMENTS_CONTROLELEMENTS_H_

#include <chrono>
#include <memory>

#include <cpsCore/Utilities/LinearAlgebra.h>

namespace Control
{

#define MUSEC_TO_SEC (1./1e6)
#define SEC_TO_MUSEC (1e6)

class IControlElement
{
public:

	virtual
	~IControlElement() = default;

	virtual FloatingType
	getValue() const = 0;

};

using Element = std::shared_ptr<IControlElement>;

}

#endif /* FLIGHTCONTROLLER_CONTROLELEMENTS_CONTROLELEMENTS_H_ */
