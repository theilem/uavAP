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

Constant::Constant(ControlFloating val) :
		val_(val)
{
}

Constant::ControlFloating
Constant::getValue()
{
	return std::isnan(val_) ? 0 : val_;
}

Constraint::Constraint(Element in, ControlFloating min, ControlFloating max) :
		in_(in), min_(min), max_(max)
{
}

Constraint::ControlFloating
Constraint::getValue()
{
	ControlFloating val = in_->getValue();
	ControlFloating ret = val > max_ ? max_ : val < min_ ? min_ : val;
	return std::isnan(ret) ? 0 : ret;
}

void
Constraint::setContraintValue(ControlFloating minmax)
{
	max_ = minmax;
	min_ = -minmax;
}

void
Constraint::setContraintValue(ControlFloating min, ControlFloating max)
{
	max_ = max;
	min_ = -min;
}

Difference::Difference(Element in1, Element in2) :
		in1_(in1), in2_(in2)
{
}

Difference::ControlFloating
Difference::getValue()
{
	ControlFloating ret = in1_->getValue() - in2_->getValue();
	return std::isnan(ret) ? 0 : ret;
}

Gain::Gain(Element in, ControlFloating gain) :
		in_(in), gain_(gain)
{
}

Gain::ControlFloating
Gain::getValue()
{
	return gain_ * in_->getValue();
}

Input::Input(ControlFloating* in) :
		in_(in)
{

}

Input::ControlFloating
Input::getValue()
{
	ControlFloating ret = *in_;
	return std::isnan(ret) ? 0 : ret;
}

Sum::Sum(Element in1, Element in2) :
		in1_(in1), in2_(in2)
{
}

Sum::ControlFloating
Sum::getValue()
{
	ControlFloating ret = in1_->getValue() + in2_->getValue();
	return std::isnan(ret) ? 0 : ret;
}

ManualSwitch::ManualSwitch(Element inTrue, Element inFalse) :
		inTrue_(inTrue), inFalse_(inFalse), state_(true)
{
}

ManualSwitch::ControlFloating
ManualSwitch::getValue()
{
	ControlFloating ret = state_ ? inTrue_->getValue() : inFalse_->getValue();
	return std::isnan(ret) ? 0 : ret;
}

void
ManualSwitch::switchTo(bool state)
{
	state_ = state;
}

}
