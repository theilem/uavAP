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
#include <cpsCore/Utilities/Time.hpp>
#include <cpsCore/Configuration/ConfigurableObject.hpp>
#include <cpsCore/Configuration/Parameter.hpp>

#include "uavAP/FlightControl/Controller/ControlElements/IEvaluableControlElement.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"

struct PIDStatus;

namespace Control
{

class Filter: public IEvaluableControlElement
{

public:

	Filter(Element in, FloatingType alpha);

	void
	evaluate() override;

	FloatingType
	getValue() const override;

	void
	setAlpha(FloatingType alpha);

private:

	Element in_;

	bool init_;

	FloatingType smoothData_;

	FloatingType alpha_;
};

class Output: public IEvaluableControlElement
{

public:

	Output(Element in, FloatingType* out);

	void
	overrideOutput(FloatingType newOutput);

	void
	setWaveform(Waveforms waveform);

	void
	setWavelength(FloatingType wavelength);

	void
	setPhase(FloatingType phase);

	void
	disableOverride();

	void
	evaluate() override;

	FloatingType
	getValue() const override;

private:

	FloatingType
	getWaveformOutput();

	Element in_;
	TimePoint start_;
	Waveforms waveform_;
	FloatingType* out_;
	bool override_;
	FloatingType overrideOut_;
	FloatingType wavelength_;
	FloatingType phase_;
};

struct PIDParameters
{
	Parameter<FloatingType> kp = {1, "kp", true};
	Parameter<FloatingType> ki = {0, "ki", false};
	Parameter<FloatingType> kd = {0, "kd", false};
	Parameter<FloatingType> imax = {std::numeric_limits<FloatingType>::max(), "imax", false};
	Parameter<FloatingType> ff = {0, "ff", false};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & kp;
		c & ki;
		c & kd;
		c & imax;
		c & ff;
	}
};

class PID: public IEvaluableControlElement, public ConfigurableObject<PIDParameters>
{
public:

	PID(Element target, Element current, const PIDParameters& params, Duration* timeDiff);

	PID(Element target, Element current, Element differential, const PIDParameters& params,
			Duration* timeDiff);

	void
	overrideTarget(FloatingType newTarget);

	void
	disableOverride();

	void
	evaluate() override;

	FloatingType
	getValue() const override;

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

	Duration* timeDiff_;

	//Internal PID state
	FloatingType targetValue_;

	FloatingType currentError_;

	FloatingType integrator_;

	FloatingType lastError_;

	FloatingType lastTarget_;

	//Latest calculated output value
	FloatingType output_;

	bool override_;
	FloatingType overrideTarget_;
};

}


#endif /* CONTROL_EVALUABLECONTROLELEMENT_H_ */
