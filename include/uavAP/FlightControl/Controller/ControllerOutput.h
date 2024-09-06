/**
 *  @file         ControllerOutput.h
 *  @author Simon Yu, Mirco Theile
 *  @date      26 June 2017
 *  @brief      UAV Autopilot Controller Output Header File
 *
 *  Description
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLEROUTPUT_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLEROUTPUT_H_

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/EnumMap.hpp>
#include <cpsCore/Utilities/NamedValue.hpp>
#include <cpsCore/Utilities/DataPresentation/detail/SerializeCustom.h>


enum class ControllerOutputs
{
    INVALID, ROLL, PITCH, YAW, THROTTLE, NUM_OUTPUT
};

enum class ControllerOutputsWaveforms
{
    INVALID, ROLL, PITCH, YAW, THROTTLE, NUM_OUTPUT
};

enum class ControllerOutputsOverrides
{
    INVALID, FREEZE, TRIM, OFFSET, NUM_OVERRIDE
};

enum class Waveforms
{
    INVALID, NONE, SINE, SQUARE, RAMP, SAWTOOTH, TRIANGULAR, NUM_WAVEFORM
};

ENUMMAP_INIT(ControllerOutputs, { {ControllerOutputs::ROLL, "roll"}, {ControllerOutputs::PITCH,
             "pitch"}, {ControllerOutputs::YAW, "yaw"}, {ControllerOutputs::THROTTLE, "throttle"} });

ENUMMAP_INIT(ControllerOutputsWaveforms, { {ControllerOutputsWaveforms::ROLL, "roll"},
             {ControllerOutputsWaveforms::PITCH, "pitch"}, {ControllerOutputsWaveforms::YAW, "yaw"},
             {ControllerOutputsWaveforms::THROTTLE, "throttle"} });

ENUMMAP_INIT(ControllerOutputsOverrides, { {ControllerOutputsOverrides::FREEZE, "freeze"},
             {ControllerOutputsOverrides::TRIM, "trim"}, {ControllerOutputsOverrides::OFFSET,
             "offset"} });

ENUMMAP_INIT(Waveforms, { {Waveforms::NONE, "none"}, {Waveforms::SINE, "sine"}, {Waveforms::SQUARE,
             "square"}, {Waveforms::RAMP, "ramp"}, {Waveforms::SAWTOOTH, "sawtooth"},
             {Waveforms::TRIANGULAR, "triangular"} });

struct ControllerOutput : SerializeCustom
{
    FloatingType rollOutput;
    FloatingType pitchOutput;
    FloatingType yawOutput;
    FloatingType throttleOutput;

    ControllerOutput() :
        rollOutput(0), pitchOutput(0), yawOutput(0), throttleOutput(-1)
    {
    }
};

struct NamedControllerOutput
{
    static constexpr auto name = "controller_output";

    NamedValue<FloatingType> rollOutput = {0.0, "roll_output"};
    NamedValue<FloatingType> pitchOutput = {0.0, "pitch_output"};
    NamedValue<FloatingType> yawOutput = {0.0, "yaw_output"};
    NamedValue<FloatingType> throttleOutput = {-1.0, "throttle_output"};

    NamedControllerOutput() = default;

    explicit
    NamedControllerOutput(const ControllerOutput& other)
    {
        rollOutput = other.rollOutput;
        pitchOutput = other.pitchOutput;
        yawOutput = other.yawOutput;
        throttleOutput = other.throttleOutput;
    }

    template <typename Parser>
    void
    parse(Parser& p) const
    {
        p & rollOutput;
        p & pitchOutput;
        p & yawOutput;
        p & throttleOutput;
    }
};

namespace dp
{
    template <class Archive, typename Type>
    inline void
    serialize(Archive& ar, ControllerOutput& t)
    {
        ar & t.rollOutput;
        ar & t.pitchOutput;
        ar & t.yawOutput;
        ar & t.throttleOutput;
    }
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLEROUTPUT_H_ */
