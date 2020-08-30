/*
 * ApExtManager.h
 *
 *  Created on: Aug 31, 2017
 *      Author: mircot
 */

#ifndef UAVAP_API_APEXTMANAGER_H_
#define UAVAP_API_APEXTMANAGER_H_

#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilter.h>
#include "uavAP/API/AutopilotAPI.hpp"
#include "uavAP/API/ap_ext/ap_ext.h"
#include "uavAP/API/ChannelMixing.h"
#include "uavAP/API/ap_ext/ApExtManagerParams.h"
#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <mutex>
#include <functional>

#include <cpsCore/cps_object>

class AggregatableAutopilotAPI;

class LinearSensorManager;
class DataHandling;

class ApExtManager
		: public AggregatableObject<AggregatableAutopilotAPI, LinearSensorManager, DataHandling>,
		  public ConfigurableObject<ApExtManagerParams>,
		  public IRunnableObject
{
public:

	static constexpr TypeId typeId = "ap_ext_manager";

	ApExtManager();

	bool
	run(RunStage stage) override;

	int
	ap_sense(const data_sample_t* sample);

	int
	ap_actuate(unsigned long* pwm, unsigned int num_channels);

	struct OutPWM
	{
		unsigned long ch[7];
	};

	struct PWMFeedback
	{
		unsigned long ch[PWM_CHS];
	};

	std::map<std::string, FloatingType>
	getMiscValues() const;

private:

	void
	onControllerOutput(const ControllerOutput& output);

	void
	onAdvancedControl(const AdvancedControl& control);

	ChannelMixing channelMixing_;

	std::mutex advancedControlMutex_;
	AdvancedControl lastAdvancedControl_;

	std::mutex outputMutex_;
	std::vector<unsigned long> outputChannels_;

	Control::LowPassFilter airspeedFilter_;

	TimePoint gpsSampleTimestamp_;
	PUBX_POS_fields lastGPSSample_{};

	TimePoint airspeedTimestamp_;
	airs_sample_t lastAirspeedSample_{};

	Vector3 lastPosition_;
	Vector3 lastVelocity_;

	std::map<std::string, FloatingType> miscValues_;
};

#endif /* UAVAP_API_APEXTMANAGER_H_ */
