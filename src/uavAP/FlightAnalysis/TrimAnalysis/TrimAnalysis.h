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
 * TrimAnalysis.h
 *
 *  Created on: Mar 19, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_TRIMANALYSIS_TRIMANALYSIS_H_
#define UAVAP_FLIGHTANALYSIS_TRIMANALYSIS_TRIMANALYSIS_H_

#include "uavAP/Core/Object/AggregatableObject.hpp"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/DataPresentation/Packet.h"

class IPC;

class TrimAnalysis : public AggregatableObject<IPC>, public IRunnableObject
{

public:

	static constexpr TypeId typeId = "trim_analysis";

	TrimAnalysis();

	static std::shared_ptr<TrimAnalysis>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

private:

	void
	onControllerOutput(const ControllerOutput& output);

	void
	onTrimAnalysis(const bool& analysis);

	void
	analysisInit();

	void
	analysisNormal(const ControllerOutput& controllerOutput);

	void
	analysisFinal();

	ControllerOutput controllerOutputTrim_;
	ControllerOutput controllerOutputCount_;
	Mutex controllerOutputMutex_;

	bool trimAnalysis_;
	bool trimAnalysisLast_;
	Mutex trimAnalysisMutex_;

	Publisher<ControllerOutput> controllerOutputTrimPublisher_;
	Subscription controllerOutputSubscription_;
	Subscription trimAnalysisSubscription_;
};

#endif /* UAVAP_FLIGHTANALYSIS_TRIMANALYSIS_TRIMANALYSIS_H_ */
