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
#include <cmath>

namespace Control
{

Constant::Constant(FloatingType val) :
		val_(val)
{
}

FloatingType
Constant::getValue() const
{
	return std::isnan(val_) ? 0 : val_;
}

Difference::Difference(Element in1, Element in2) :
		in1_(in1), in2_(in2)
{
}

FloatingType
Difference::getValue() const
{
	FloatingType ret = in1_->getValue() - in2_->getValue();
	return std::isnan(ret) ? 0 : ret;
}

Gain::Gain(Element in, FloatingType gain) :
		in_(in), gain_(gain)
{
}

FloatingType
Gain::getValue() const
{
	return gain_ * in_->getValue();
}

Input::Input(const FloatingType* in) :
		in_(in)
{

}

FloatingType
Input::getValue() const
{
	FloatingType ret = *in_;
	return std::isnan(ret) ? 0 : ret;
}

Sum::Sum(Element in1, Element in2) :
		in1_(in1), in2_(in2)
{
}

FloatingType
Sum::getValue() const
{
	FloatingType ret = in1_->getValue() + in2_->getValue();
	return std::isnan(ret) ? 0 : ret;
}

ManualSwitch::ManualSwitch(Element inTrue, Element inFalse) :
		inTrue_(inTrue), inFalse_(inFalse), state_(true)
{
}

FloatingType
ManualSwitch::getValue() const
{
	FloatingType ret = state_ ? inTrue_->getValue() : inFalse_->getValue();
	return std::isnan(ret) ? 0 : ret;
}

void
ManualSwitch::switchTo(bool state)
{
	state_ = state;
}

CustomFunction::CustomFunction(Element input, std::function<FloatingType
		(FloatingType)> function) : input_(input), function_(function)
{
}

FloatingType
CustomFunction::getValue() const
{
	return function_(input_->getValue());
}

CustomFunction2::CustomFunction2(Element input, Element input2, std::function<FloatingType
		(FloatingType, FloatingType)> function) : input_(input), input2_(input2), function_(function)
{
}

FloatingType
CustomFunction2::getValue() const
{
	return function_(input_->getValue(), input2_->getValue());
}

}
