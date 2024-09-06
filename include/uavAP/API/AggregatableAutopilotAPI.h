//
// Created by seedship on 6/28/20.
//

#ifndef UAVAP_AGGREGATABLEAUTOPILOTAPI_H
#define UAVAP_AGGREGATABLEAUTOPILOTAPI_H

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <cpsCore/Utilities/IPC/Publisher.h>

#include "uavAP/Core/Frames/VehicleOneFrame.h"
#include "uavAP/API/IAutopilotAPI.h"
//#include "uavAP/API/AggregatableAutopilotAPIParams.h"

struct SensorData;

struct ServoData;

struct PowerData;

struct AdvancedControl;

struct ControllerOutput;

class IScheduler;

class IPC;

class AggregatableAutopilotAPI : public AggregatableObject<IPC, IScheduler>, public IRunnableObject, public IAutopilotAPI
{
public:
	static constexpr TypeId typeId = "agg_autopilot_api";

	AggregatableAutopilotAPI();

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot) override;

	boost::signals2::connection
	subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot) override;

	void
	setSensorData(const SensorData& sd) override;

	void
	setServoData(const ServoData& sd) override;

	void
	setPowerData(const PowerData& pd) override;

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
