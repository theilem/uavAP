//
// Created by mirco on 12.12.24.
//

#include "uavAP/ExternalPlanner/PythonAPI/uavAPI.h"
#include "uavAP/ExternalPlanner/PythonAPI/PythonAPIHelper.hpp"
#include "cpsCore/Synchronization/SimpleRunner.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/FlightControl/LocalPlanner/ForwardingLocalPlanner/ForwardingLocalPlanner.h"


uavAPI::uavAPI()
{
	aggregator_ = PythonAPIHelper::createAggregation();

	auto runner = SimpleRunner(aggregator_);
	runner.runAllStages();

	auto ipc = aggregator_.getOne<IPC>();
	sensorSubscription_ = ipc->subscribe<SensorData>("sensor_data", [this](const SensorData& data)
	{
		sensorData_ = data;
	});
}

const SensorData&
uavAPI::getSensorData() const
{
	return sensorData_;
}

uavAPI::~uavAPI()
{
	sensorSubscription_.cancel();

	aggregator_.clear();
}

void
uavAPI::setSplineSegment(const QuarticSpline& spline)
{
	Trajectory trajectory;
	trajectory.pathSections.push_back(std::make_shared<QuarticSpline>(spline));
	auto lp = aggregator_.getOne<ForwardingLocalPlanner>();
	lp->setTrajectory(trajectory);
}
