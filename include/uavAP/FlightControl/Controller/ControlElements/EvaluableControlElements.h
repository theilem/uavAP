/*
 * EvaluableControlElement.h
 *
 *  Created on: Jun 19, 2017
 *      Author: mircot
 */

#ifndef CONTROL_EVALUABLECONTROLELEMENTS_H_
#define CONTROL_EVALUABLECONTROLELEMENTS_H_

#include <iostream>
#include <cpsCore/Utilities/Time.hpp>
#include <cpsCore/Configuration/ConfigurableObject.hpp>
#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/Utilities/Filtering.hpp>

#include "uavAP/FlightControl/Controller/ControlElements/IEvaluableControlElement.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/Core/OverrideHandler/OverridableValue.hpp"

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

	void
	reset();

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

	Output(Element in, FloatingType* out, FloatingType alpha);

	void
	evaluate() override;

	FloatingType
	getValue() const override;

	OverridableValue<FloatingType>&
	getOutputOverridableValue();

	OverridableValue<FloatingType>&
	getSaveTrimOverridableValue();

	OverridableValue<FloatingType>&
	getApplyTrimOverridableValue();

	FloatingType&
	getTrimAlpha();

private:


	Element in_;
	FloatingType* out_;
	OverridableValue<FloatingType> outputOverride_;
	OverridableValue<FloatingType> saveTrimOverride_;
	OverridableValue<FloatingType> applyTrimOverride_;
	MovingAverageValue<FloatingType> trim_;
};

struct PIDParameters
{
	Parameter<FloatingType> kp = {1, "kp", true};
	Parameter<FloatingType> ki = {0, "ki", false};
	Parameter<FloatingType> kd = {0, "kd", false};
	Parameter<FloatingType> imax = {std::numeric_limits<FloatingType>::max(), "imax", false};
	Parameter<FloatingType> ff = {0, "ff", false};
	Parameter<bool> isAngle = {false, "is_angle", false};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & kp;
		c & ki;
		c & kd;
		c & imax;
		c & ff;
		c & isAngle;
	}

	bool
	operator ==(const PIDParameters& other) const
	{
		return kp == other.kp && ki == other.ki && kd == other.kd && imax == other.imax && ff == other.ff
				&& isAngle == other.isAngle;
	}
};

class PID: public IEvaluableControlElement, public ConfigurableObject<PIDParameters>
{
public:

	PID(Element target, Element current, const PIDParameters& params, Duration* timeDiff);

	PID(Element target, Element current, Element differential, const PIDParameters& params,
			Duration* timeDiff);

	void
	applyOverride(bool enable, FloatingType value);

	void
	applyMaintain(bool enable);

	void
	evaluate() override;

	FloatingType
	getValue() const override;

	PIDStatus
	getStatus() const;

	MaintainableValue<FloatingType>
	getMaintainableTarget();

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
	MaintainableValue<FloatingType> targetValue_;

	FloatingType currentError_;

	FloatingType integrator_;

	FloatingType lastError_;

	FloatingType lastTarget_;

	//Latest calculated output value
	FloatingType output_;
};

}


#endif /* CONTROL_EVALUABLECONTROLELEMENT_H_ */
