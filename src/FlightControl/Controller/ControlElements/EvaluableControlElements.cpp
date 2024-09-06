/*
 * EvaluableControlElement.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: uav
 */

#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"

namespace Control
{
    Filter::Filter(Element in, FloatingType alpha) :
        in_(in), init_(true), smoothData_(0), alpha_(alpha)
    {
    }

    void
    Filter::evaluate()
    {
        if (init_)
        {
            init_ = false;
            smoothData_ = in_->getValue();
            return;
        }

        smoothData_ = alpha_ * in_->getValue() + (1 - alpha_) * smoothData_;
    }

    FloatingType
    Filter::getValue() const
    {
        return smoothData_;
    }

    void
    Filter::setAlpha(FloatingType alpha)
    {
        alpha_ = alpha;
    }

    void
    Filter::reset()
    {
        init_ = true;
    }

    Output::Output(Element in, FloatingType* out) : Output(in, out, 1.0)
    {
    }

    Output::Output(Element in, FloatingType* out, FloatingType alpha):
        in_(in), out_(out), trim_(1.0, alpha)
    {
        saveTrimOverride_ = 0;
        applyTrimOverride_ = 0;
    }

    //void
    //Output::overrideOutput(FloatingType newOutput)
    //{
    //	overrideOut_ = newOutput;
    //	start_ = Clock::now();
    //	override_ = true;
    //}
    //
    //void
    //Output::setWaveform(Waveforms waveform)
    //{
    //	waveform_ = waveform;
    //}
    //
    //void
    //Output::setWavelength(FloatingType wavelength)
    //{
    //	wavelength_ = wavelength;
    //}
    //
    //void
    //Output::setPhase(FloatingType phase)
    //{
    //	phase_ = phase;
    //}
    //
    //void
    //Output::disableOverride()
    //{
    //	override_ = false;
    //	start_ = TimePoint();
    //	waveform_ = Waveforms::NONE;
    //	wavelength_ = 0;
    //	phase_ = 0;
    //}

    void
    Output::evaluate()
    {
        outputOverride_ = in_->getValue();
        if (applyTrimOverride_() != 0.0)
            *out_ = outputOverride_ + trim_;
        else
            *out_ = outputOverride_;

        if (saveTrimOverride_() != 0.0)
        {
            trim_ = outputOverride_;
        }
    }

    FloatingType
    Output::getValue() const
    {
        return outputOverride_;
    }

    OverridableValue<FloatingType>&
    Output::getOutputOverridableValue()
    {
        return outputOverride_;
    }

    OverridableValue<FloatingType>&
    Output::getSaveTrimOverridableValue()
    {
        return saveTrimOverride_;
    }

    OverridableValue<FloatingType>&
    Output::getApplyTrimOverridableValue()
    {
        return applyTrimOverride_;
    }

    FloatingType&
    Output::getTrimAlpha()
    {
        return trim_.getAlpha();
    }

    //
    //FloatingType
    //Output::getWaveformOutput()
    //{
    //	FloatingType waveformOutput = 0;
    //
    //	switch (waveform_)
    //	{
    //	case Waveforms::NONE:
    //	{
    //		waveformOutput = overrideOut_;
    //
    //		break;
    //	}
    //	case Waveforms::SINE:
    //	{
    //		TimePoint current = Clock::now();
    //		FloatingType time = std::chrono::duration_cast<Microseconds>(current - start_).count()
    //				/ 1000.0;
    //		FloatingType period = (2 * M_PI) / wavelength_;
    //
    //		waveformOutput = overrideOut_ * sin(period * (time + phase_));
    //
    //		break;
    //	}
    //	case Waveforms::SQUARE:
    //	{
    //		TimePoint current = Clock::now();
    //		FloatingType time = std::chrono::duration_cast<Microseconds>(current - start_).count()
    //				/ 1000.0;
    //		FloatingType period = (2 * M_PI) / wavelength_;
    //		FloatingType sine = sin(period * (time + phase_));
    //
    //		if (sine > 0)
    //		{
    //			waveformOutput = overrideOut_;
    //		}
    //		else if (sine == 0)
    //		{
    //			waveformOutput = 0;
    //		}
    //		else if (sine < 0)
    //		{
    //			waveformOutput = -overrideOut_;
    //		}
    //
    //		break;
    //	}
    //	case Waveforms::RAMP:
    //	{
    //		TimePoint current = Clock::now();
    //		FloatingType time = std::chrono::duration_cast<Microseconds>(current - start_).count()
    //				/ 1000.0;
    //		FloatingType time_mod = fmod(time + phase_, wavelength_);
    //		FloatingType slope = (2 * overrideOut_) / wavelength_;
    //		FloatingType intercept = -overrideOut_;
    //
    //		waveformOutput = slope * time_mod + intercept;
    //
    //		break;
    //	}
    //	case Waveforms::SAWTOOTH:
    //	{
    //		TimePoint current = Clock::now();
    //		FloatingType time = std::chrono::duration_cast<Microseconds>(current - start_).count()
    //				/ 1000.0;
    //		FloatingType time_mod = fmod(time + phase_, wavelength_);
    //		FloatingType slope = -(2 * overrideOut_) / wavelength_;
    //		FloatingType intercept = overrideOut_;
    //
    //		waveformOutput = slope * time_mod + intercept;
    //
    //		break;
    //	}
    //	case Waveforms::TRIANGULAR:
    //	{
    //		waveformOutput = overrideOut_;
    //
    //		break;
    //	}
    //	default:
    //	{
    //		waveformOutput = overrideOut_;
    //
    //		break;
    //	}
    //	}
    //
    //	return waveformOutput;
    //}
    //
    //void
    //Output::applyOverride(bool enable, FloatingType value)
    //{
    //	override_ = enable;
    //	overrideOut_ = value;
    //
    //}

    PID::PID(Element target, Element current, const PIDParameters& p, Duration* timeDiff) :
        ConfigurableObject(p), target_(target), current_(current), timeDiff_(timeDiff), targetValue_(0.05,
            0), currentError_(0), integrator_(0), lastError_(0), lastTarget_(0), output_(0)
    {
    }

    PID::PID(Element target, Element current, Element derivative, const PIDParameters& p,
             Duration* timeDiff) :
        ConfigurableObject(p), target_(target), current_(current), derivative_(derivative), timeDiff_(
            timeDiff), targetValue_(0.05, 0), currentError_(0), integrator_(0), lastError_(0), lastTarget_(
            0), output_(0)
    {
    }

    void
    PID::applyOverride(bool enable, FloatingType value)
    {
        float target = params.isAngle() ? degToRad(value) : value;

        targetValue_.applyOverride(enable, target);
    }

    void
    PID::applyMaintain(bool enable)
    {
        targetValue_.maintainValue(enable);
    }

    void
    PID::evaluate()
    {
        if (!current_ || !target_)
            return;

        output_ = 0.0;
        targetValue_ = target_->getValue();
        currentError_ = targetValue_ - current_->getValue();
        addProportionalControl();
        addIntegralControl();
        addDifferentialControl();
        addFeedForwardControl();
        lastError_ = currentError_;
        lastTarget_ = targetValue_;
    }

    FloatingType
    PID::getValue() const
    {
        return std::isnan(output_) ? 0 : output_;
    }

    void
    PID::addProportionalControl()
    {
        output_ += params.kp() * currentError_;
    }

    void
    PID::addIntegralControl()
    {
        if (params.ki() == 0. || !timeDiff_)
            return;

        auto timeDiffus = std::chrono::duration_cast<Microseconds>(*timeDiff_).count();
        if (timeDiffus > 0)
            integrator_ += currentError_ * timeDiffus * MUSEC_TO_SEC;

        integrator_ = std::clamp(integrator_, -params.imax(), params.imax());

        output_ += params.ki() * integrator_;
    }

    void
    PID::addDifferentialControl()
    {
        if (params.kd() == 0.)
            return;

        auto timeDiffus = std::chrono::duration_cast<Microseconds>(*timeDiff_).count();
        if (derivative_)
        {
            FloatingType targetDer = 0;

            if (!std::isnan(lastTarget_) && timeDiff_
                && timeDiffus > 0.)
                targetDer = (targetValue_ - lastTarget_)
                    / (timeDiffus * MUSEC_TO_SEC);

            output_ += params.kd() * (targetDer - derivative_->getValue());
        }
        else if (!std::isnan(lastError_) && timeDiff_
            && timeDiffus > 0.)
        {
            FloatingType derivative = (currentError_ - lastError_)
                / (timeDiffus * MUSEC_TO_SEC);
            output_ += params.kd() * derivative;
        }
    }

    PIDStatus
    PID::getStatus() const
    {
        PIDStatus status{targetValue_, current_->getValue(), integrator_};
        return status;
    }

    void
    PID::addFeedForwardControl()
    {
        if (params.ff() != 0)
        {
            output_ += params.ff() * targetValue_;
        }
    }

    MaintainableValue<FloatingType>
    PID::getMaintainableTarget()
    {
        return targetValue_;
    }
}
