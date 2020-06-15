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

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/IPC/Publisher.h>

#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilter.h>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <cpsCore/Utilities/LockTypes.hpp>
#include "uavAP/MissionControl/WindAnalysis/WindAnalysisStatus.h"
#include "uavAP/MissionControl/WindAnalysis/WindAnalysisParams.h"

class IPC;
class Packet;
class SensorData;

class WindAnalysis: public AggregatableObject<IPC>,
		public IRunnableObject,
		public ConfigurableObject<WindAnalysisParams>
{
public:

	static constexpr TypeId typeId = "wind_analysis";

	WindAnalysis() = default;

	bool
	run(RunStage stage) override;

	WindInfo
	getWindInfo() const;

	WindAnalysisStatus
	getWindAnalysisStatus() const;

	void
	setWindAnalysisStatus(const WindAnalysisStatus& windAnalysisStatus);

	template <typename Config>
	inline void
	configureParams(Config& c)
	{
		ParameterRef<Control::LowPassFilterGeneric<Vector3>> windFilter(windFilter_, "wind_filter", false);

		params.configure(c);
		c & windFilter;
	}

private:

	void
	onSensorData(const SensorData& sensorData);

	Subscription sensorDataSubscription_;
	Publisher<WindInfo> windInfoPublisher_;

	TimePoint timeStamp_;

	WindInfo windInfo_;
	mutable Mutex windInfoMutex_;

	WindAnalysisStatus windAnalysisStatus_;
	mutable Mutex windAnalysisStatusMutex_;

	Control::LowPassFilterGeneric<Vector3> windFilter_;
};

#endif /* UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSIS_H_ */
