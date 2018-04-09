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
 * ControlElements.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: mircot
 */

#include "uavAP/FlightControl/Controller/ControlElements/ControlElements.h"

namespace Control
{

Constant::Constant(double val) :
		val_(val)
{
}

double
Constant::getValue()
{
	return val_;
}

Constraint::Constraint(Element in, double min, double max) :
		in_(in),
		min_(min),
		max_(max)
{
}

double
Constraint::getValue()
{
	double val = in_->getValue();
	return val > max_ ? max_ : val < min_ ? min_ : val;
}

void
Constraint::setContraintValue(double minmax)
{
	max_ = minmax;
	min_ = -minmax;
}

void
Constraint::setContraintValue(double min, double max)
{
	max_ = max;
	min_ = -min;
}

Difference::Difference(Element in1, Element in2) :
		in1_(in1),
		in2_(in2)
{
}

double
Difference::getValue()
{
	return in1_->getValue() - in2_->getValue();
}

Gain::Gain(Element in, double gain) :
		in_(in),
		gain_(gain)
{
}

double
Gain::getValue()
{
	return gain_ * in_->getValue();
}

Input::Input(double* in) :
		in_(in)
{

}

double
Input::getValue()
{
	return *in_;
}

Sum::Sum(Element in1, Element in2) :
		in1_(in1),
		in2_(in2)
{
}

double
Sum::getValue()
{
	return in1_->getValue() + in2_->getValue();
}

ManualSwitch::ManualSwitch(Element inTrue, Element inFalse) :
		inTrue_(inTrue),
		inFalse_(inFalse),
		state_(true)
{
}

double
ManualSwitch::getValue()
{
	return state_ ? inTrue_->getValue() : inFalse_->getValue();
}

void
ManualSwitch::switchTo(bool state)
{
	state_ = state;
}

}

