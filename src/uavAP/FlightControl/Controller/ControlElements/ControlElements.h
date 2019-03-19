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
 *  Created on: Jun 19, 2017
 *      Author: mircot
 */

#ifndef CONTROL_CONTROLELEMENTS_H_
#define CONTROL_CONTROLELEMENTS_H_

#include "uavAP/FlightControl/Controller/ControlElements/IControlElement.h"

namespace Control
{

class Constant: public IControlElement
{
public:
	Constant(double val);

	double
	getValue() override;

private:

	double val_;
};

class Constraint: public IControlElement
{

public:

	Constraint(Element in, double min, double max);

	Constraint(Element in, double min, double max, double hardMin, double hardMax);

	double
	getValue() override;

	void
	setHardContraintValue(double hardMinmax);

	void
	setHardContraintValue(double hardMin, double hardMax);

	void
	setContraintValue(double minmax);

	void
	setContraintValue(double min, double max);

	void
	overrideContraintValue(double overrideMinmax);

	void
	overrideContraintValue(double overrideMin, double overrideMax);

	void
	disableOverride();

private:

	Element in_;
	double min_;
	double max_;
	double hardMin_;
	double hardMax_;
	bool override_;
	double overrideMin_;
	double overrideMax_;
};

class Difference: public IControlElement
{
public:

	Difference(Element in1, Element in2);

	double
	getValue() override;

private:

	Element in1_;
	Element in2_;
};

class Gain: public IControlElement
{

public:

	Gain(Element in, double gain);

	double
	getValue() override;

private:

	Element in_;

	double gain_;

};

class Input: public IControlElement
{
public:

	Input(double* in);

	double
	getValue() override;

private:

	double* in_;
};

class Sum: public IControlElement
{
public:

	Sum(Element in1, Element in2);

	double
	getValue() override;

private:

	Element in1_;
	Element in2_;
};

class ManualSwitch: public IControlElement
{
public:

	ManualSwitch(Element inTrue, Element inFalse);

	double
	getValue() override;

	void
	switchTo(bool state);

private:

	Element inTrue_;
	Element inFalse_;

	bool state_;
};

}

#endif /* CONTROL_CONTROLELEMENTS_H_ */
