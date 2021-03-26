//
// Created by seedship on 1/30/21.
//

#ifndef UAVAP_PITCHSTATESPACECONTROLLER_H
#define UAVAP_PITCHSTATESPACECONTROLLER_H

#include <cpsCore/cps_object>
#include <cpsCore/Aggregation/IAggregatableObject.h>
#include <uavAP/FlightControl/Controller/PIDController/IPIDController.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceCascade.h"


class DataHandling;
class ISensingIO;
class IActuationIO;
class OverrideHandler;

class PitchStateSpaceController	: public AggregatableObject<IScheduler, ISensingIO, IActuationIO, DataHandling,
		OverrideHandler>,
		public IPIDController,
		public ConfigurableObject<PlaceholderParams>,
		public IRunnableObject
{
public:
	PitchStateSpaceController();

	void
	setControllerTarget(const ControllerTarget& target) override;

	bool
	run(RunStage stage) override;

	ControllerOutput
	getControllerOutput() override;

	bool
	configure(const Configuration& config) override;

	static constexpr TypeId typeId = "statespace_pitch";


private:

	void
	tunePID(const PIDTuning& tune);

	PitchStateSpaceCascade cascade_;
	SensorData sensorDataENU_;
	SensorData sensorDataNED_;
	ControllerTarget target_;
	ControllerOutput output_;

	Mutex cascadeMutex_;
};


#endif //UAVAP_PITCHSTATESPACECONTROLLER_H
