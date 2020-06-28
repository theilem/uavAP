//
// Created by seedship on 6/28/20.
//

#ifndef UAVAP_AGGREGATABLEAUTOPILOTAPI_H
#define UAVAP_AGGREGATABLEAUTOPILOTAPI_H

#include <cpsCore/Aggregation/AggregatableObject.hpp>
#include <cpsCore/Synchronization/IRunnableObject.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <cpsCore/Utilities/IPC/Publisher.h>

#include "uavAP/Core/Frames/VehicleOneFrame.h"

class SensorData;
class ServoData;
class PowerData;
class AdvancedControl;
class ControllerOutput;
class IPC;

#include <boost/signals2.hpp>

using OnControllerOut = boost::signals2::signal<void(const ControllerOutput&)>;
using OnAdvancedControl = boost::signals2::signal<void(const AdvancedControl&)>;

class AggregatableAutopilotAPI : public AggregatableObject<IScheduler, IPC>, IRunnableObject
{
public:
	static constexpr TypeId typeId = "autopilotapi";

	AggregatableAutopilotAPI();

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot);

	boost::signals2::connection
	subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot);

	void
	setSensorData(const SensorData& sd);

	void
	setServoData(const ServoData& sd);

	void
	setPowerData(const PowerData& pd);

	bool
	run(RunStage stage) override;

private:

	void
	tryConnectControllerOut();

	void
	tryConnectAdvancedControl();

	void
	tryConnectLocalFrame();

	void
	onControllerOut(const ControllerOutput& control);

	void
	onAdvancedControl(const AdvancedControl& control);

	void
	onLocalFrame(const VehicleOneFrame& frame);


	OnControllerOut onControllerOut_;
	OnAdvancedControl onAdvancedControl_;

	Publisher<SensorData> sensorDataPublisher_;
	Publisher<ServoData> servoDataPublisher_;
	Publisher<PowerData> powerDataPublisher_;
	Subscription controllerOutSubscription_;
	Subscription advancedControlSubscription_;
	Subscription localFrameSubscription_;

	VehicleOneFrame localFrame_;
};


#endif //UAVAP_AGGREGATABLEAUTOPILOTAPI_H
