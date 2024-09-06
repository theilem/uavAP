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
 * ControlEnvironment.cpp
 *
 *  Created on: Jun 16, 2017
 *      Author: mircot
 */

#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"

namespace Control
{

ControlEnvironment::ControlEnvironment(const TimePoint* timeStamp) :
		timeStamp_(timeStamp)
{
}

void
ControlEnvironment::evaluate()
{
	//Calculate timeDiff
	if (timeStamp_)
	{
		if (lastTimeStamp_.time_since_epoch() == Duration(0))
		{
			timeDiff_ = Microseconds(0);
		}
		else
		{
			timeDiff_ = *timeStamp_ - lastTimeStamp_;
		}
		lastTimeStamp_ = *timeStamp_;
	}
	else
	{
		CPSLOG_WARN << "No timestamp in ControlEnvironment";
		timeDiff_ = Microseconds(0);
	}

	//Iterate over Control Elements that need Evaluation
	for (auto it : evaluableControlElements_)
	{
		it->evaluate();
	}
}

std::shared_ptr<Input>
ControlEnvironment::addInput(const FloatingType* in)
{
	return std::make_shared<Input>(in);
}

std::shared_ptr<Filter>
ControlEnvironment::addFilter(Element in, FloatingType alpha)
{
	auto filter = std::make_shared<Filter>(in, alpha);
	evaluableControlElements_.push_back(filter);
	return filter;
}

std::shared_ptr<Output>
ControlEnvironment::addOutput(Element in, FloatingType* out)
{
	auto output = std::make_shared<Output>(in, out);
	evaluableControlElements_.push_back(output);
	return output;
}

std::shared_ptr<Sum>
ControlEnvironment::addSum(Element in1, Element in2)
{
	return std::make_shared<Sum>(in1, in2);
}

std::shared_ptr<Difference>
ControlEnvironment::addDifference(Element in1, Element in2)
{
	return std::make_shared<Difference>(in1, in2);
}

std::shared_ptr<Gain>
ControlEnvironment::addGain(Element in, FloatingType gain)
{
	return std::make_shared<Gain>(in, gain);
}

std::shared_ptr<Constant>
ControlEnvironment::addConstant(FloatingType val)
{
	return std::make_shared<Constant>(val);
}

std::shared_ptr<ManualSwitch>
ControlEnvironment::addManualSwitch(Element inTrue, Element inFalse)
{
	return std::make_shared<ManualSwitch>(inTrue, inFalse);
}

std::shared_ptr<Constraint<>>
ControlEnvironment::addConstraint(Element in, FloatingType min, FloatingType max)
{
	return std::make_shared<Constraint<>>(in, min, max);
}

std::shared_ptr<Constraint<>>
ControlEnvironment::addConstraint(Element in, FloatingType min, FloatingType max, FloatingType hardMin, FloatingType hardMax)
{
	return std::make_shared<Constraint<>>(in, min, max);
}

std::shared_ptr<PID>
ControlEnvironment::addPID(Element target, Element current, const PIDParameters& params)
{
	auto pid = std::make_shared<PID>(target, current, params, &timeDiff_);
	evaluableControlElements_.push_back(pid);
	return pid;
}

std::shared_ptr<PID>
ControlEnvironment::addPID(Element target, Element current, Element derivative,
		const PIDParameters& params)
{
	auto pid = std::make_shared<PID>(target, current, derivative, params, &timeDiff_);
	evaluableControlElements_.push_back(pid);
	return pid;
}

const Duration*
ControlEnvironment::getTimeDiff() const
{
	return &timeDiff_;
}

void
ControlEnvironment::addEvaluable(EvaluableElement element)
{
	evaluableControlElements_.push_back(element);
}
} /* namespace Control */

