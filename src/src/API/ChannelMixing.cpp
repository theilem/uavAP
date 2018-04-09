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
 * ChannelMixing.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include "uavAP/API/ChannelMixing.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"

ChannelMixing::ChannelMixing() :
		airplane_(true), numOfOutputChannel_(0)
{
}

bool
ChannelMixing::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	boost::property_tree::ptree channelConfig;
	pm.add("servo", channelConfig, true);
	pm.add("num_output_channel", numOfOutputChannel_, true);
	pm.add("airplane", airplane_, false);

	if (!pm.map())
	{
		APLOG_ERROR << "ChannelMixing configuration failed.";
		return false;
	}

	if (airplane_)
	{
		return configureAirplaneChannel(channelConfig);
	}

	return configureHelicopterChannel(channelConfig);;
}

std::vector<double>
ChannelMixing::mixChannels(const ControllerOutput& controllerOut)
{
	std::vector<double> output(numOfOutputChannel_, 0.0);
	if (airplane_)
	{
		if (servoConfig_.size() < (int)Airplane::NUM_AIRPLANE_CHANNEL)
		{
			APLOG_ERROR << "Servo config size does not match.";
			return std::vector<double>();
		}
		auto channel = mixAirplane(controllerOut);

		for (int i = (int) Airplane::AILERONL; i < (int) Airplane::NUM_AIRPLANE_CHANNEL; i++)
		{
			ServoConfig& servo = servoConfig_[i];
			if (servo.mapped)
			{
				if (servo.reversed)
					output[servo.outChannel] = channel.ch[i] * -1;
				else
					output[servo.outChannel] = channel.ch[i];
			}
		}
	}
	return output;
}

ControllerOutput
ChannelMixing::unmixChannels(const std::vector<double>& channels)
{
	AirplaneChannel airplane;

	for (int i = (int) Airplane::AILERONL; i < (int) Airplane::NUM_AIRPLANE_CHANNEL; i++)
	{
		ServoConfig& servo = servoConfig_[i];
		if (servo.mapped)
			airplane.ch[i] = channels[servo.outChannel] * (servo.reversed ? -1 : 1);
	}

	//Fill in unmapped channels
	for (int i = (int) Airplane::AILERONL; i < (int) Airplane::NUM_AIRPLANE_CHANNEL; i++)
	{
		ServoConfig& servo = servoConfig_[i];
		if (!servo.mapped)
		{
			switch (static_cast<Airplane>(i))
			{
			case Airplane::AILERONL :
				airplane.ch[i] = -1 * airplane.ch[(int)Airplane::AILERONR];
				break;
			case Airplane::AILERONR :
				airplane.ch[i] = -1 * airplane.ch[(int)Airplane::AILERONL];
				break;
			case Airplane::ELEVATORL :
				airplane.ch[i] = airplane.ch[(int)Airplane::ELEVATORR];
				break;
			case Airplane::ELEVATORR :
				airplane.ch[i] = airplane.ch[(int)Airplane::ELEVATORL];
				break;
			case Airplane::FLAPL :
				airplane.ch[i] = airplane.ch[(int)Airplane::FLAPR];
				break;
			case Airplane::FLAPR :
				airplane.ch[i] = airplane.ch[(int)Airplane::FLAPL];
				break;
			default:
				break;
			}
		}
	}

	return unmixAirplane(airplane);
}

ChannelMixing::AirplaneChannel
ChannelMixing::mixAirplane(const ControllerOutput& controllerOut)
{
	AirplaneChannel airplane;
	airplane.ch[(int) Airplane::AILERONL] = controllerOut.rollOutput;
	airplane.ch[(int) Airplane::AILERONR] = -controllerOut.rollOutput;
	airplane.ch[(int) Airplane::ELEVATORL] = controllerOut.pitchOutput;
	airplane.ch[(int) Airplane::ELEVATORR] = controllerOut.pitchOutput;
	airplane.ch[(int) Airplane::FLAPL] = controllerOut.flapOutput;
	airplane.ch[(int) Airplane::FLAPR] = controllerOut.flapOutput;
	airplane.ch[(int) Airplane::RUDDER] = controllerOut.yawOutput;
	airplane.ch[(int) Airplane::THROTTLE] = controllerOut.throttleOutput;
	return airplane;
}

ControllerOutput
ChannelMixing::unmixAirplane(const AirplaneChannel& channel)
{
	ControllerOutput control;
	control.rollOutput = channel.ch[(int) Airplane::AILERONL];
	control.pitchOutput = channel.ch[(int) Airplane::ELEVATORL];
	control.flapOutput = channel.ch[(int) Airplane::FLAPL];
	control.yawOutput = channel.ch[(int) Airplane::RUDDER];
	control.throttleOutput = channel.ch[(int) Airplane::THROTTLE];
	control.collectiveOutput = 0;
	return control;
}

ChannelMixing::ServoConfig::ServoConfig(const boost::property_tree::ptree& config) :
		mapped(false), reversed(false), outChannel(0)
{
	PropertyMapper pm(config);
	pm.add("reversed", reversed, false);
	if (pm.add("channel", outChannel, false))
		mapped = true;

}

bool
ChannelMixing::configureAirplaneChannel(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree aileronlConf;
	boost::property_tree::ptree aileronrConf;
	boost::property_tree::ptree elevatorlConf;
	boost::property_tree::ptree elevatorrConf;
	boost::property_tree::ptree flaplConf;
	boost::property_tree::ptree flaprConf;
	boost::property_tree::ptree rudderConf;
	boost::property_tree::ptree throttleConf;
	pm.add("aileron_l", aileronlConf, true);
	pm.add("aileron_r", aileronrConf, true);
	pm.add("elevator_l", elevatorlConf, true);
	pm.add("elevator_r", elevatorrConf, true);
	pm.add("flap_l", flaplConf, true);
	pm.add("flap_r", flaprConf, true);
	pm.add("rudder", rudderConf, true);
	pm.add("throttle", throttleConf, true);

	if (!pm.map())
	{
		APLOG_ERROR << "ChannelMixing missing servo configuration.";
		return false;
	}

	servoConfig_.push_back(ServoConfig(aileronlConf));
	servoConfig_.push_back(ServoConfig(aileronrConf));
	servoConfig_.push_back(ServoConfig(elevatorlConf));
	servoConfig_.push_back(ServoConfig(elevatorrConf));
	servoConfig_.push_back(ServoConfig(flaplConf));
	servoConfig_.push_back(ServoConfig(flaprConf));
	servoConfig_.push_back(ServoConfig(rudderConf));
	servoConfig_.push_back(ServoConfig(throttleConf));
	return true;
}
bool
ChannelMixing::configureHelicopterChannel(const boost::property_tree::ptree& config)
{
	return true;
}

bool
ChannelMixing::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "ChannelMixing: IPC missing.";
			return true;
		}
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "ChannelMixing: DataPresentation missing.";
			return true;
		}

		auto ipc = ipc_.get();
		channelMixPublisher_ = ipc->publishPackets("data_ch_com");

		break;
	}
	default:
		break;
	}
	return false;
}

void
ChannelMixing::notifyAggregationOnUpdate(Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
}

std::shared_ptr<ChannelMixing>
ChannelMixing::create(const boost::property_tree::ptree& config)
{
	auto chMix = std::make_shared<ChannelMixing>();
	chMix->configure(config);
	return chMix;
}
