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
/**
 *  @file         DataHandlingIO.h
 *  @author Simon Yu
 *  @date      26 July 2017
 *  @brief      UAV Autopilot Flight Control Data Handling IO Header File
 *
 *  Description
 */

#ifndef UAVAP_FLIGHTCONTROL_DATAHANDLING_FLIGHTCONTROLDATAHANDLING_H_
#define UAVAP_FLIGHTCONTROL_DATAHANDLING_FLIGHTCONTROLDATAHANDLING_H_

#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/FlightControl/Controller/ManeuverPIDController/ManeuverPIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDController.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IPC/IPC.h"

class IGlobalPlanner;
class ILocalPlanner;
class IController;
class IScheduler;
enum class Content;
enum class Target;

template<typename C, typename T>
class IDataPresentation;
struct ControllerOutput;

class FlightControlDataHandling: public IAggregatableObject, public IRunnableObject
{
public:
	FlightControlDataHandling();

	static std::shared_ptr<FlightControlDataHandling>
	create(const boost::property_tree::ptree& configuration);

	bool
	configure(const boost::property_tree::ptree& configuration);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

private:
	void
	collectAndSend();

	void
	receiveAndDistribute(const Packet& packet);

	void
	collectAndSendSensorData(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendTrajectory(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendPIDStatus(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendLocalPlannerStatus(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	tunePID(const PIDTuning& params);

	void
	tuneLocalPlanner(const LocalPlannerParams& params);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<LinearLocalPlanner> localPlanner_;
	ObjectHandle<IController> controller_;
	ObjectHandle<SensingActuationIO> sensActIO_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;
	Subscription flightControlSubscription_;
	Publisher publisher_;

	Duration period_;
};

#endif /* UAVAP_FLIGHTCONTROL_DATAHANDLING_FLIGHTCONTROLDATAHANDLING_H_ */
