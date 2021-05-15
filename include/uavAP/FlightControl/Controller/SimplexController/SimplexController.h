//
// Created by seedship on 4/30/21.
//

#ifndef UAVAP_SIMPLEXCONTROLLER_H
#define UAVAP_SIMPLEXCONTROLLER_H


#include <cpsCore/cps_object>
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDController.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceCascade.h"
#include "uavAP/FlightControl/Controller/SimplexController/SimplexControllerParams.h"

class IScheduler;
class ISensingIO;
class IActuationIO;
class DataHandling;
class OverrideHandler;

class SimplexController :  public AggregatableObject<IScheduler, ISensingIO, IActuationIO, DataHandling,
		OverrideHandler>,
						   public IPIDController,
						   public ConfigurableObject<SimplexControllerParams>,
						   public IRunnableObject
{
public:
	static constexpr TypeId typeId = "simplex";

	SimplexController();

	void
	setControllerTarget(const ControllerTarget& target) override;

	ControllerOutput
	getControllerOutput() override;

	bool
	configure(const Configuration& config) override;

	bool
	run(RunStage stage) override;

	VectorN<5>
	generateState() const;

private:

	void
	tunePID(const PIDTuning& tune);

	Vector2
	_generateU(const ControllerOutput& t) const;

	void
	executeSafetyControl(const VectorN<5>& state);

	PitchStateSpaceCascade cascade_;
	SensorData sensorDataENU_;
	SensorData sensorDataNED_;
	ControllerTarget target_;
	ControllerOutput output_;

	TimePoint switchTime_;
	bool safetyController_;

	mutable Mutex cascadeMutex_;
};


#endif //UAVAP_SIMPLEXCONTROLLER_H
