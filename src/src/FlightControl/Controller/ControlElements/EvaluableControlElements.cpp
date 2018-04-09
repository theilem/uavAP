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
 * EvaluableControlElement.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: uav
 */

#include "uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h"
#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

namespace Control
{

Output::Output(Element in, double* out) :
		in_(in), out_(out)
{
}

void
Output::evaluate()
{
	*out_ = in_->getValue();
}

double
Output::getValue()
{
	return in_->getValue();
}

PID::PID(Element target, Element current, const Parameters& params, Duration* timeDiff) :
		target_(target), current_(current), params_(params), timeDiff_(timeDiff), currentError_(0), integrator_(
				0), lastError_(0), output_(0)
{

}

PID::PID(Element target, Element current, Element derivative, const Parameters& params,
		Duration* timeDiff) :
		target_(target), current_(current), derivative_(derivative), params_(params), timeDiff_(
				timeDiff), currentError_(0), integrator_(0), lastError_(0), output_(0)
{

}

void
PID::setControlParameters(const Parameters& g)
{
	params_ = g;
	integrator_ = 0;
}

void
PID::evaluate()
{
	if (!current_ || !target_)
		return;

	output_ = 0.0;
	currentError_ = target_->getValue() - current_->getValue();
	addProportionalControl();
	addIntegralControl();
	addDifferentialControl();
	addFeedForwardControl();
	lastError_ = currentError_;
}

double
PID::getValue()
{
	return output_;
}

void
PID::addProportionalControl()
{
	output_ += params_.kp * currentError_;
}

void
PID::addIntegralControl()
{
	if (params_.ki == 0. || !timeDiff_)
		return;

	integrator_ += currentError_ * timeDiff_->total_microseconds() * MUSEC_TO_SEC;

	if (integrator_ > 0)
		integrator_ = std::min(integrator_, params_.imax);
	else
		integrator_ = std::max(integrator_, -params_.imax);

	output_ += params_.ki * integrator_;
}

void
PID::addDifferentialControl()
{
	if (params_.kd == 0.)
		return;

	if (derivative_)
	{
		//Take the negative derivative value to counter acceleration towards the target
		output_ -= params_.kd * derivative_->getValue();
	}
	else if (!isnanf(lastError_) && timeDiff_ && timeDiff_->total_microseconds() > 0.)
	{
		double derivative = (currentError_ - lastError_)
				/ (timeDiff_->total_microseconds() * MUSEC_TO_SEC);
		output_ += params_.kd * derivative;
	}
}

PIDStatus
PID::getStatus()
{
	PIDStatus status;
	status.target = target_->getValue();
	status.value = current_->getValue();
	return status;
}

void
PID::addFeedForwardControl()
{
	if (params_.ff != 0)
	{
		output_ += params_.ff * target_->getValue();
	}
}

}
