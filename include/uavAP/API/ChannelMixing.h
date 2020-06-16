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
#include <Eigen/Dense>

#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include <cpsCore/Configuration/Configuration.hpp>

struct ControllerOutput;

class ChannelMixing
{
public:

	ChannelMixing();

	bool
	configure(const Configuration& config);

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
	getMapping(const Configuration& config);

	int numOfOutputChannel_;

	Eigen::MatrixXd mixingMatrix_;

	std::map<ThrowsControl, Mapping> mapping_;

	std::map<CamberControl, Eigen::ArrayXd> camberOffsets_;

	std::map<SpecialControl, Eigen::ArrayXd> specialOffsets_;

	Eigen::ArrayXd channelMin_;
	Eigen::ArrayXd channelMax_;
};

#endif /* UAVAP_SIMULATION_CHANNELMIXING_H_ */
