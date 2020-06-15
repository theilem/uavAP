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
 * ICondition.h
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_ICONDITION_H_
#define UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_ICONDITION_H_

#include <boost/property_tree/ptree.hpp>
#include <vector>
#include <functional>

class ConditionManager;

class ICondition
{

public:

	static constexpr auto typeId = "condition";

	virtual
	~ICondition() = default;

	using ConditionTrigger = std::function<void(int)>;

	virtual void
	activate(ConditionManager*, const ConditionTrigger&) = 0;

	virtual void
	deactivate() = 0;
};

#endif /* UAVAP_MISSIONCONTROL_CONDITIONMANAGER_CONDITION_ICONDITION_H_ */
