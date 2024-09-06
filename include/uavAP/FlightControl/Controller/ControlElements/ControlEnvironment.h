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
 * ControlEnvironment.h
 *
 *  Created on: Jun 16, 2017
 *      Author: mircot
 */

#ifndef FLIGHTCONTROLLER_CONTROLELEMENTS_CONTROLENVIRONMENT_H_
#define FLIGHTCONTROLLER_CONTROLELEMENTS_CONTROLENVIRONMENT_H_

#include "uavAP/FlightControl/Controller/ControlElements/Control.h"
#include <vector>
#include <memory>

namespace Control
{

class ControlEnvironment
{
public:

	explicit ControlEnvironment(const TimePoint* timeStamp);

	virtual
	~ControlEnvironment() = default;

	void
	evaluate();

	using Element = std::shared_ptr<IControlElement>;
	using EvaluableElement = std::shared_ptr<IEvaluableControlElement>;

	std::shared_ptr<Input>
	addInput(const FloatingType* in);

	std::shared_ptr<Filter>
	addFilter(Element in, FloatingType alpha);

	std::shared_ptr<Output>
	addOutput(Element in, FloatingType* out);

	std::shared_ptr<Sum>
	addSum(Element in1, Element in2);

	std::shared_ptr<Difference>
	addDifference(Element in1, Element in2);

	std::shared_ptr<Gain>
	addGain(Element in, FloatingType gain);

	std::shared_ptr<Constant>
	addConstant(FloatingType val);

	std::shared_ptr<ManualSwitch>
	addManualSwitch(Element inTrue, Element inFalse);

	std::shared_ptr<Constraint<>>
	addConstraint(Element in, FloatingType min, FloatingType max);

	std::shared_ptr<Constraint<>>
	addConstraint(Element in, FloatingType min, FloatingType max, FloatingType hardMin, FloatingType hardMax);

	std::shared_ptr<PID>
	addPID(Element target, Element current, const PIDParameters& params);

	std::shared_ptr<PID>
	addPID(Element target, Element current, Element derivative, const PIDParameters& params);

	const Duration*
	getTimeDiff() const;

	void
	addEvaluable(EvaluableElement element);

private:

	Duration timeDiff_;

	TimePoint lastTimeStamp_;

	const TimePoint* timeStamp_;

	std::vector<EvaluableElement> evaluableControlElements_;

};

} /* namespace Control */

#endif /* FLIGHTCONTROLLER_CONTROLELEMENTS_CONTROLENVIRONMENT_H_ */
