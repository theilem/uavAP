//
// Created by seedship on 7/23/21.
//

#ifndef UAVAP_RSLQRCONTROLLER_H
#define UAVAP_RSLQRCONTROLLER_H

#include <cpsCore/cps_object>
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/RSLQRController/RSLQRControllerParams.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"

class IScheduler;

class ISensingIO;

class IActuationIO;

class DataHandling;

class OverrideHandler;

enum class DataRequest;

class RSLQRController: public IController,
					   public ConfigurableObject<RSLQRControllerParams>,
					   public AggregatableObject<IScheduler, ISensingIO, IActuationIO, DataHandling,
							   OverrideHandler>,
					   public IRunnableObject
{

public:
	static constexpr TypeId typeId = "rslqr_controller";

	RSLQRController();

	bool
	run(RunStage stage) override;

	void
	setControllerTarget(const ControllerTarget& target) override;

	VectorN<12>
	getState();

	Optional<PIDParams>
	getDummyPIDParams(const DataRequest& request);

	std::map<PIDs, PIDStatus>
	getStatus() const;

private:

	mutable Mutex lock_;
	SensorData sd_ned_;

	Control::ControlEnvironment controlEnv_;
	std::unordered_map<ControllerOutputs, std::shared_ptr<Control::Output>> ctrlEnvOutputs_;

	ControllerTarget ct_;
	ControllerOutput co_;

	VectorN<4> stateOut_;

	void
	populateControlEnv();

};


#endif //UAVAP_RSLQRCONTROLLER_H
