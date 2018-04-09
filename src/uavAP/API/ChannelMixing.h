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
 * ChannelMixing.h
 *
 *  Created on: Jul 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_SIMULATION_CHANNELMIXING_H_
#define UAVAP_SIMULATION_CHANNELMIXING_H_
#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include <vector>

struct ControllerOutput;
class IPC;
enum class Content;
enum class Target;

template<typename C, typename T>
class IDataPresentation;

class ChannelMixing: public IAggregatableObject, public IRunnableObject
{
public:

	ChannelMixing();

	static std::shared_ptr<ChannelMixing>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	std::vector<double>
	mixChannels(const ControllerOutput& controllerOut);

	ControllerOutput
	unmixChannels(const std::vector<double>& channels);

	enum class Airplane
	{
		AILERONL = 0,
		AILERONR,
		ELEVATORL,
		ELEVATORR,
		FLAPL,
		FLAPR,
		RUDDER,
		THROTTLE,
		NUM_AIRPLANE_CHANNEL
	};

	struct AirplaneChannel
	{
		double ch[(int) Airplane::NUM_AIRPLANE_CHANNEL];
	};

	struct ServoConfig
	{
		bool mapped;
		bool reversed;
		unsigned int outChannel;
		//Mapping as well

		ServoConfig(const boost::property_tree::ptree& config);
	};

	AirplaneChannel
	mixAirplane(const ControllerOutput& controllerOut);

	ControllerOutput
	unmixAirplane(const AirplaneChannel& controllerOut);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

private:

	bool
	configureAirplaneChannel(const boost::property_tree::ptree& config);

	bool
	configureHelicopterChannel(const boost::property_tree::ptree& config);

	bool airplane_;
	int numOfOutputChannel_;

	std::vector<ServoConfig> servoConfig_;

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;

	Publisher channelMixPublisher_;

};

#endif /* UAVAP_SIMULATION_CHANNELMIXING_H_ */
