/*
 * AutopilotAPI.hpp
 *
 *  Created on: Jul 12, 2018
 *      Author: mircot
 */

#ifndef UAVAP_API_AUTOPILOTAPI_HPP_
#define UAVAP_API_AUTOPILOTAPI_HPP_

#include <cpsCore/Aggregation/Aggregator.h>

#include "uavAP/API/AggregatableAutopilotAPI.h"

class AutopilotAPI : public IAutopilotAPI
{
public:

	void
	configure(const Configuration& config);


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

	/**
	 * @brief Establishes connection to Autopilot shared memory instances. Creates sensor_data in SM and
	 * 		  subscribes on actuation and advanced_control SM.
	 */
	void
	initialize();

private:
	Aggregator aggregator_;
};


#endif /* UAVAP_API_AUTOPILOTAPI_HPP_ */
