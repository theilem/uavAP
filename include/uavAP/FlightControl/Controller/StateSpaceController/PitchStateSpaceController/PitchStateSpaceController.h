//
// Created by seedship on 1/30/21.
//

#ifndef UAVAP_PITCHSTATESPACECONTROLLER_H
#define UAVAP_PITCHSTATESPACECONTROLLER_H

#include <cpsCore/cps_object>
#include <uavAP/FlightControl/Controller/PIDController/IPIDController.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include "uavAP/Core/SensorData.h"
#include "PitchStateSpaceCascade.h"


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

	static constexpr TypeId typeId = "statespace_pitch";

	PitchStateSpaceController();

	void
	setControllerTarget(const ControllerTarget& target) override;

	bool
	run(RunStage stage) override;

	ControllerOutput
	getControllerOutput() override;

	VectorN<6>
	getState() const;

	FloatingType
	getUTrim() const;

	FloatingType
	getThetaTrim() const;

	bool
	configure(const Configuration& config) override;

private:

	void
	tunePID(const PIDTuning& tune);

	PitchStateSpaceCascade cascade_;
	SensorData sensorDataENU_;
	SensorData sensorDataNED_;
	ControllerTarget target_;
	ControllerOutput output_;

	mutable Mutex cascadeMutex_;
};


#endif //UAVAP_PITCHSTATESPACECONTROLLER_H
