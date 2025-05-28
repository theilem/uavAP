//
// Created by Mirco Theile on 24/5/25.
//

#ifndef DOUBLEINTEGRATOR_H
#define DOUBLEINTEGRATOR_H

#include <cpsCore/cps_object>

#include "cpsCore/Utilities/Scheduler/Event.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"

class IScheduler;

struct DoubleIntegratorSimParams
{
    Parameter<int> period = {10, "period", true};
    Parameter<Vector3> initPosition = {Vector3(0, 0, 150), "init_position", true};
    Parameter<FloatingType> defaultVelocity = {20., "default_velocity", true};

    // Physical properties
    Parameter<FloatingType> mass = {9.3, "mass", true}; // kg
    Parameter<Vector3> inertia = {Vector3(1.5, 2.0, 2.5), "inertia", true}; // kg·m² (estimated)

    // Propulsion
    Parameter<FloatingType> maxThrust = {100.0, "max_thrust", true}; // N

    // Aerodynamic drag coefficients
    Parameter<FloatingType> linDragCoeff = {0.5, "lin_drag_coeff", true};
    Parameter<FloatingType> quadDragCoeff = {0.05, "quad_drag_coeff", true};
    Parameter<Vector3> rotDragCoeff = {Vector3(0.2, 0.2, 0.2), "rot_drag_coeff", true};

    // Lift model
    Parameter<FloatingType> liftCoeff = {0.6, "lift_coeff", true}; // dimensionless
    Parameter<Vector3> wingLiftArm = {Vector3(0.0, 1.15, 0.0), "wing_lift_arm", true}; // m
    Parameter<Vector3> tailLiftArm = {Vector3(-1.0, 0.0, 0.0), "tail_lift_arm", true}; // m
    Parameter<FloatingType> tailLiftScale = {0.2, "tail_lift_scale", true}; // dimensionless

    // Coupling coefficient for yaw torque from roll + lift
    Parameter<FloatingType> yawCouplingCoeff = {0.5, "yaw_coupling_coeff", true};

    template <typename Configurator>
    void
    configure(Configurator& c)
    {
        c & period;
        c & initPosition;
        c & defaultVelocity;

        c & mass;
        c & inertia;
        c & maxThrust;

        c & linDragCoeff;
        c & quadDragCoeff;
        c & rotDragCoeff;

        c & liftCoeff;
        c & wingLiftArm;
        c & tailLiftArm;
        c & tailLiftScale;

        c & yawCouplingCoeff;
    }
};


class FixedWingSim : public ISensingIO,
                     public IActuationIO,
                     public IRunnableObject,
                     public AggregatableObject<IScheduler, ITimeProvider>,
                     public ConfigurableObject<DoubleIntegratorSimParams>
{
public:
    static constexpr auto typeId = "fixed_wing_sim";
    FixedWingSim() = default;
    ~FixedWingSim() override = default;
    bool
    run(RunStage stage) override;
    void
    setControllerOutput(const ControllerOutput& out) override;
    SensorData
    getSensorData() const override;
    OnSensorDataConnection
    subscribeOnSensorData(const OnSensorDataSlot& slot) override;
    void
    reset(const Vector3& position, const Vector3& direction);

private:
    void
    step();

    SensorData sensorData_;
    ControllerOutput controllerOutput_;
    OnSensorData onSensorData_;
    Event stepEvent_;
    TimePoint lastStepTime_;
};

#endif //DOUBLEINTEGRATOR_H
