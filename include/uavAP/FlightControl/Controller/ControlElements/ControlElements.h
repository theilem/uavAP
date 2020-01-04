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

#include <cpsCore/Configuration/ConfigurableObject.hpp>
#include <cpsCore/Utilities/LinearAlgebra.h>
#include "uavAP/FlightControl/Controller/ControlElements/IControlElement.h"

namespace Control
{

class Constant : public IControlElement
{
public:
	Constant(FloatingType val);

	FloatingType
	getValue() const override;

private:

	FloatingType val_;
};

template<typename Type>
struct ConstraintParams
{
	Parameter<Type> min =
			{
					{}, "min", true};
	Parameter<Type> max =
			{
					{}, "max", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & min;
		c & max;
	}
};

template<typename Type = FloatingType>
class Constraint : public ConfigurableObject<ConstraintParams<Type>>, public IControlElement
{

public:

	Constraint(Element in) :
			in_(in), override_(false)
	{
	}

	Constraint(Element in, Type min, Type max) :
			in_(in), override_(false)
	{
		this->params.min = min;
		this->params.max = max;
	}

	FloatingType
	getValue() const override
	{
		if (override_)
			return in_->getValue() > overrideMax_ ?
				   overrideMax_ : (in_->getValue() < overrideMin_ ? overrideMin_ : in_->getValue());

		return in_->getValue() > this->params.max() ?
			   this->params.max() :
			   (in_->getValue() < this->params.min() ? this->params.min() : in_->getValue());
	}

	void
	setConstraintValue(Type min, Type max)
	{
		this->params.min = min;
		this->params.max = max;
	}

	void
	setConstraintValue(Type minmax)
	{
		this->params.min = -minmax;
		this->params.max = minmax;
	}

	void
	overrideConstraintValue(Type overrideMinmax)
	{
		overrideMin_ = -overrideMinmax;
		overrideMax_ = overrideMinmax;
		override_ = true;
	}

	void
	overrideConstraintValue(Type overrideMin, Type overrideMax)
	{
		overrideMin_ = overrideMin;
		overrideMax_ = overrideMax;
		override_ = true;
	}

	void
	disableOverride()
	{
		override_ = false;
	}

	Element in_;
	bool override_;
	Type overrideMin_;
	Type overrideMax_;
};

class Difference : public IControlElement
{
public:

	Difference(Element in1, Element in2);

	FloatingType
	getValue() const override;

private:

	Element in1_;
	Element in2_;
};

class Gain : public IControlElement
{

public:

	Gain(Element in, FloatingType gain);

	FloatingType
	getValue() const override;

private:

	Element in_;

	FloatingType gain_;

};

class Input : public IControlElement
{
public:

	Input(const FloatingType* in);

	FloatingType
	getValue() const override;

private:

	const FloatingType* in_;
};

class Sum : public IControlElement
{
public:

	Sum(Element in1, Element in2);

	FloatingType
	getValue() const override;

private:

	Element in1_;
	Element in2_;
};

class ManualSwitch : public IControlElement
{
public:

	ManualSwitch(Element inTrue, Element inFalse);

	FloatingType
	getValue() const override;

	void
	switchTo(bool state);

private:

	Element inTrue_;
	Element inFalse_;

	bool state_;
};

class CustomFunction : public IControlElement
{
public:

	CustomFunction(Element input, std::function<FloatingType
			(FloatingType)> function);

	FloatingType
	getValue() const override;

private:

	Element input_;
	std::function<FloatingType
			(FloatingType)> function_;
};

class CustomFunction2 : public IControlElement
{
public:

	CustomFunction2(Element input, Element input2, std::function<FloatingType
			(FloatingType, FloatingType)> function);

	FloatingType
	getValue() const override;

private:

	Element input_;
	Element input2_;
	std::function<FloatingType
			(FloatingType, FloatingType)> function_;
};

}

#endif /* CONTROL_CONTROLELEMENTS_H_ */
