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

#include <vector>
#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <Eigen/Dense>

#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightControl/Controller/AdvancedControl.h"

struct ControllerOutput;

class ChannelMixing
{
public:

	ChannelMixing();

	bool
	configure(const boost::property_tree::ptree& config);

	Eigen::VectorXd
	mixChannels(const ControllerOutput& controllerOut);

	std::vector<unsigned int>
	mapChannels(const ControllerOutput& out, const AdvancedControl& advanced);

private:

	struct Mapping
	{
		Eigen::ArrayXd negThrows;
		Eigen::ArrayXd center;
		Eigen::ArrayXd posThrows;
	};

	Mapping
	getMapping(const boost::property_tree::ptree& config);

	int numOfOutputChannel_;

	Eigen::MatrixXd mixingMatrix_;

	std::map<ThrowsControl, Mapping> mapping_;

	std::map<CamberControl, Eigen::ArrayXd> camberOffsets_;

	std::map<SpecialControl, Eigen::ArrayXd> specialOffsets_;

	Eigen::ArrayXd channelMin_;
	Eigen::ArrayXd channelMax_;
};

#endif /* UAVAP_SIMULATION_CHANNELMIXING_H_ */
