/*
 * AutopilotAPI.hpp
 *
 *  Created on: Jul 12, 2018
 *      Author: mircot
 */

#ifndef UAVAP_API_AUTOPILOTAPI_HPP_
#define UAVAP_API_AUTOPILOTAPI_HPP_
#include <uavAP/Core/Object/Aggregator.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/Core/IPC/Publisher.h>

#include <boost/signals2.hpp>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/Core/IPC/Subscription.h>


class AutopilotAPI
{
public:

	AutopilotAPI();

	using OnControllerOut = boost::signals2::signal<void (const ControllerOutput&)>;

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot);

	using OnAdvancedControl = boost::signals2::signal<void (const AdvancedControl&)>;

	boost::signals2::connection
	subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot);

	void
	setSensorData(const SensorData& sd);

	/**
	 * @brief Establishes connection to Autopilot shared memory instances. Creates sensor_data in SM and
	 * 		  subscribes on actuation and advanced_control SM.
	 */
	void
	initialize();

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

	Aggregator aggregator_;

	OnControllerOut onControllerOut_;

	OnAdvancedControl onAdvancedControl_;

	Publisher<SensorData> sensorDataPublisher_;
	Subscription controllerOutSubscription_;
	Subscription advancedControlSubscription_;
	Subscription localFrameSubscription_;

	VehicleOneFrame localFrame_;


};


#endif /* UAVAP_API_AUTOPILOTAPI_HPP_ */
