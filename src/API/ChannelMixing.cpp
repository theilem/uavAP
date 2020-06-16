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
#include "uavAP/FlightControl/Controller/ControllerOutput.h"

ChannelMixing::ChannelMixing() :
		numOfOutputChannel_(0)
{
}

bool
ChannelMixing::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	Configuration channelMixing;
	Configuration channelMapping;
	Configuration camberOffset;
	Configuration specialOffset;
	Configuration channelContraints;
	pm.add("channel_mixing", channelMixing, true);
	pm.add("channel_mapping", channelMapping, true);
	pm.add("camber_offset", camberOffset, true);
	pm.add("special_control", specialOffset, true);
	pm.add("channel_constraints", channelContraints, true);
	pm.add<int>("num_output_channel", numOfOutputChannel_, true);

	if (!pm.map())
	{
		CPSLOG_ERROR << "Failed to Configure ChannelMixing.";
		return false;
	}

	/* Create mixing matrix */
	PropertyMapper<Configuration> pmMix(channelMixing);
	Eigen::VectorXd roll, pitch, yaw, throttle;
	pmMix.add("roll_out", roll, true);
	pmMix.add("pitch_out", pitch, true);
	pmMix.add("yaw_out", yaw, true);
	pmMix.add("throttle_out", throttle, true);

	mixingMatrix_.resize(numOfOutputChannel_, 4);
	mixingMatrix_.col(0) = roll;
	mixingMatrix_.col(1) = pitch;
	mixingMatrix_.col(2) = yaw;
	mixingMatrix_.col(3) = throttle;

	/* Load mappings */
	for (const auto& it : channelMapping)
	{
		auto throwsEnum = EnumMap<ThrowsControl>::convert(it.first);

		if (throwsEnum == ThrowsControl::INVALID)
		{
			CPSLOG_ERROR << "Invalid Throws Control " << it.first;
			continue;
		}

		mapping_.insert(std::make_pair(throwsEnum, getMapping(it.second)));
	}

	/* Load camber */
	PropertyMapper<Configuration> camberPm(camberOffset);

	for (const auto& it : camberOffset)
	{
		auto camberEnum = EnumMap<CamberControl>::convert(it.first);

		if (camberEnum == CamberControl::INVALID)
		{
			CPSLOG_ERROR << "Invalid Camber Control " << it.first;
			continue;
		}

		Eigen::ArrayXd offset;
		camberPm.add(it.first, offset, true);
		camberOffsets_.insert(std::make_pair(camberEnum, offset));
	}

	/* Load special */
	PropertyMapper<Configuration> specialPm(specialOffset);

	for (const auto& it : specialOffset)
	{
		auto specialEnum = EnumMap<SpecialControl>::convert(it.first);

		if (specialEnum == SpecialControl::INVALID)
		{
			CPSLOG_ERROR << "Invalid Special Control " << it.first;
			continue;
		}

		Eigen::ArrayXd offset;
		specialPm.add(it.first, offset, true);
		specialOffsets_.insert(std::make_pair(specialEnum, offset));
	}

	/* Load constraints */
	PropertyMapper<Configuration> constraintPm(channelContraints);
	constraintPm.add("min", channelMin_, true);
	constraintPm.add("max", channelMax_, true);

	return pmMix.map() && camberPm.map() && specialPm.map() && constraintPm.map();
}

Eigen::VectorXd
ChannelMixing::mixChannels(const ControllerOutput& out)
{
	Eigen::Vector4d controlVector(out.rollOutput, out.pitchOutput, out.yawOutput,
			out.throttleOutput);

	return mixingMatrix_ * controlVector;
}

std::vector<unsigned int>
ChannelMixing::mapChannels(const ControllerOutput& out, const AdvancedControl& advanced)
{
	auto mix = mixChannels(out);

	auto throws = mapping_.find(advanced.throwsSelection);
	if (throws == mapping_.end())
	{
		CPSLOG_ERROR << "Throws not available. Set to first mapping.";
		throws = mapping_.begin();
	}

	const auto& negThrows = throws->second.negThrows;
	const auto& posThrows = throws->second.posThrows;
	const auto& center = throws->second.center;

	Eigen::ArrayXd result = mix.array().min(0) * negThrows + mix.array().max(0) * posThrows
			+ center;

	if (advanced.camberSelection != CamberControl::NORMAL)
	{
		auto camber = camberOffsets_.find(advanced.camberSelection);
		if (camber == camberOffsets_.end())
		{
			CPSLOG_ERROR << "camber not available. Set to first camber.";
			camber = camberOffsets_.begin();
		}

		result = result + (camber->second * advanced.camberValue);
	}

	if (advanced.specialSelection != SpecialControl::NONE)
	{
		auto special = specialOffsets_.find(advanced.specialSelection);
		if (special == specialOffsets_.end())
		{
			CPSLOG_ERROR << "special not available. Set to first special.";
			special = specialOffsets_.begin();
		}

		result = result + (special->second * advanced.specialValue);
	}

	result = result.max(channelMin_).min(channelMax_);

	std::vector<unsigned int> vec;
	for (int i = 0; i < result.size(); i++)
	{
		vec.push_back(round(result[i]));
	}

	return vec;

}

ChannelMixing::Mapping
ChannelMixing::getMapping(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	Eigen::ArrayXd min, max, center;
	pm.add("min", min, true);
	pm.add("max", max, true);
	pm.add("center", center, true);

	Mapping map;
	map.negThrows = center - min;
	map.posThrows = max - center;
	map.center = center;

	return map;
}
