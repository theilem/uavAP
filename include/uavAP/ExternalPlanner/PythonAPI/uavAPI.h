//
// Created by mirco on 12.12.24.
//

#ifndef UAVAP_UAVAPI_H
#define UAVAP_UAVAPI_H


#include "cpsCore/Aggregation/Aggregator.h"
#include "cpsCore/Utilities/IPC/Subscription.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/QuarticSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "cpsCore/Utilities/IPC/Publisher.h"

class uavAPI
{
public:
	uavAPI();

	~uavAPI();

	const SensorData&
	getSensorData() const;

	void
	setSplineSegment(const QuarticSpline& spline);

private:

	Aggregator aggregator_;

	Subscription sensorSubscription_;
	Publisher<Trajectory> trajectoryPublisher_;

	SensorData sensorData_;
};

#endif //UAVAP_UAVAPI_H
