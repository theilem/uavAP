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
 * EvaluableControlElement.h
 *
 *  Created on: Jun 19, 2017
 *      Author: mircot
 */

#ifndef CONTROL_EVALUABLECONTROLELEMENTS_H_
#define CONTROL_EVALUABLECONTROLELEMENTS_H_

#include <cmath>
#include <iostream>
#include "uavAP/Core/Time.h"
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/ControlElements/IEvaluableControlElement.h"

struct PIDStatus;

namespace Control
{

class Output: public IEvaluableControlElement
{

public:

	Output(Element in, double* out);

	void
	evaluate() override;

	double
	getValue() override;

private:

	Element in_;
	double* out_;

};

class PID: public IEvaluableControlElement
{
public:

	struct Parameters
	{
		double kp, ki, kd, imax, ff;

		Parameters() :
				kp(0), ki(0), kd(0), imax(INFINITY), ff(0)
		{
		}

		bool
		configure(const boost::property_tree::ptree& p)
		{
			PropertyMapper pm(p);
			pm.add("kp", kp, true);
			pm.add("ki", ki, false);
			pm.add("kd", kd, false);
			pm.add("imax", imax, false);
			pm.add("ff", ff, false);
			return pm.map();
		}
	};

	PID(Element target, Element current, const Parameters& params, Duration* timeDiff);

	PID(Element target, Element current, Element differential, const Parameters& params,
			Duration* timeDiff);

	/*
	 * Setters for control gains
	 */

	void
	setControlParameters(const Parameters& params);

	void
	evaluate() override;

	double
	getValue() override;

	PIDStatus
	getStatus();

private:

	void
	addProportionalControl();

	void
	addIntegralControl();

	void
	addDifferentialControl();

	void
	addFeedForwardControl();

	//Inputs to the PID
	Element target_;

	Element current_;

	Element derivative_;

	//PID Parameters
	Parameters params_;

	Duration* timeDiff_;

	//Internal PID state

	double currentError_;

	double integrator_;

	double lastError_;

	//Latest calculated output value
	double output_;
};

}

#endif /* CONTROL_EVALUABLECONTROLELEMENT_H_ */
