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
 * WindAnalysis.h
 *
 *  Created on: Oct 19, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSIS_H_
#define UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSIS_H_

#include <uavAP/Core/Object/AggregatableObject.hpp>
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/PropertyMapper/Configuration.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/MissionControl/WindAnalysis/WindAnalysisStatus.h"

class IPC;
class Packet;

class WindAnalysis: public AggregatableObject<IPC>, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "wind_analysis";

	WindAnalysis() = default;

	static std::shared_ptr<WindAnalysis>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

	WindInfo
	getWindInfo() const;

	WindAnalysisStatus
	getWindAnalysisStatus() const;

	void
	setWindAnalysisStatus(const WindAnalysisStatus& windAnalysisStatus);

private:

	Publisher<WindInfo> windInfoPublisher_;

	WindInfo windInfo_;
	mutable Mutex windInfoMutex_;

	WindAnalysisStatus windAnalysisStatus_;
	mutable Mutex windAnalysisStatusMutex_;
};

#endif /* UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSIS_H_ */
