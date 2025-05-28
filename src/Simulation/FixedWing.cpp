//
// Created by Mirco Theile on 24/5/25.
//
#include <algorithm>

#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>

#include "uavAP/Simulation/FixedWing.h"

bool
FixedWingSim::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSetAll())
                return true;
            sensorData_.hasGPSFix = true;
            break;
        }
    case RunStage::NORMAL:
        {
            auto sched = get<IScheduler>();
            stepEvent_ = sched->schedule([this]() { step(); }, Milliseconds(params.period()),
                                         Milliseconds(params.period()),
                                         "double_integrator_step");
            sched->schedule([this]() { lastStepTime_ = get<ITimeProvider>()->now(); }, Milliseconds(0),
                            "set_init_time");
            break;
        }
    default: ;
    }
    return false;
}

void
FixedWingSim::setControllerOutput(const ControllerOutput& out)
{
    controllerOutput_ = out;
}

SensorData
FixedWingSim::getSensorData() const
{
    return sensorData_;
}

ISensingIO::OnSensorDataConnection
FixedWingSim::subscribeOnSensorData(const OnSensorDataSlot& slot)
{
    return onSensorData_.connect(slot);
}

void
FixedWingSim::reset(const Vector3& position, const Vector3& direction)
{
    sensorData_.position = position;
    sensorData_.velocity = direction.normalized() * params.defaultVelocity();
    sensorData_.attitude = {0.,0., std::atan2(direction.y(), direction.x())};
    sensorData_.angularRate = Vector3(0., 0., 0.);
    sensorData_.airSpeed = params.defaultVelocity();
    onSensorData_(sensorData_);
}

void
FixedWingSim::step()
{
    // Time handling
    auto tp = get<ITimeProvider>();
    auto now = tp->now();
    auto dt = std::chrono::duration<FloatingType>(now - lastStepTime_).count();
    lastStepTime_ = now;

    if (dt <= 0.0)
        return;

    // Extract parameters
    const auto m = params.mass();
    const auto I = params.inertia(); // inertia.x = Ixx, y = Iyy, z = Izz
    const auto maxThrust = params.maxThrust();
    const auto k_lin = params.linDragCoeff();
    const auto k_quad = params.quadDragCoeff();
    const auto k_rot = params.rotDragCoeff();

    const auto liftCoeff = params.liftCoeff();
    const auto wingLiftArm = params.wingLiftArm();
    const auto tailLiftArm = params.tailLiftArm();
    const auto tailLiftScale = params.tailLiftScale();
    const auto yawCouplingCoeff = params.yawCouplingCoeff();

    // Extract state
    Vector3& pos = sensorData_.position;
    Vector3& vel = sensorData_.velocity;
    Vector3& att = sensorData_.attitude;      // Euler angles: roll, pitch, yaw
    Vector3& omega = sensorData_.angularRate; // Roll rate, pitch rate, yaw rate

    // --- Compute body-frame forces ---

    // Thrust: forward in +X body
    FloatingType throttle = (controllerOutput_.throttleOutput + 1.0) / 2.0; // [-1, 1] -> [0, 1]
    Vector3 thrustBody(throttle * maxThrust, 0, 0);

    // Drag (linear and quadratic) in body frame
    Vector3 dragBody = -k_lin * vel - k_quad * vel.cwiseProduct(vel.cwiseAbs());

    // Lift: crude model, upward in Z body frame
    FloatingType liftForce = liftCoeff * vel.norm(); // crude: lift ∝ airspeed
    Vector3 liftBody(0, 0, liftForce);

    // Net force in body frame
    Vector3 totalForceBody = thrustBody + liftBody + dragBody;

    // --- Rotation from body to world ---
    auto eulerToRotationMatrix = [](const Vector3& euler) -> Matrix3 {
        AngleAxis rollAngle(euler.x(), Vector3::UnitX());
        AngleAxis pitchAngle(euler.y(), Vector3::UnitY());
        AngleAxis yawAngle(euler.z(), Vector3::UnitZ());
        return Matrix3(yawAngle * pitchAngle * rollAngle);
    };

    Eigen::Matrix3f R_bw = eulerToRotationMatrix(att);

    // Total force in world frame
    Vector3 gravity(0, 0, -9.81 * m);
    Vector3 totalForceWorld = R_bw * totalForceBody + gravity;

    // Linear acceleration and integration
    Vector3 acc = totalForceWorld / m;
    vel += acc * dt;
    pos += vel * dt;

    // --- Torques and angular dynamics ---

    // Control surface outputs as torques (unitless)
    FloatingType rollInput = controllerOutput_.rollOutput;
    FloatingType pitchInput = controllerOutput_.pitchOutput;
    FloatingType yawInput = controllerOutput_.yawOutput;
    Vector3 torqueInput(rollInput, pitchInput, yawInput); // simplified control torque

    // Induced lift torques
    Vector3 wingLift(0, 0, liftForce);          // upward in Z-body
    Vector3 tailLift(0, 0, liftForce * tailLiftScale);    // tail lift is smaller

    Vector3 wingTorque = wingLiftArm.cross(wingLift);
    Vector3 tailTorque = tailLiftArm.cross(tailLift);
    Vector3 liftTorque = wingTorque + tailTorque;

    // Rotational damping
    Vector3 angularDrag = -k_rot.cwiseProduct(omega);

    // Total torque: control + lift + damping
    Vector3 totalTorque = torqueInput + liftTorque + angularDrag;

    // --- Yaw torque from rolled lift vector ---
    FloatingType roll = att.x(); // roll in radians
    totalTorque.z() += yawCouplingCoeff * liftForce * sin(roll) * wingLiftArm.y();

    // Angular acceleration: τ = I * α ⇒ α = τ / I
    Vector3 angularAcc(
        totalTorque.x() / I.x(),
        totalTorque.y() / I.y(),
        totalTorque.z() / I.z()
    );

    // Integrate angular velocity and attitude
    omega += angularAcc * dt;
    att += omega * dt;


    // Update derived sensor values
    sensorData_.acceleration = R_bw.transpose() * acc; // body frame
    sensorData_.airSpeed = vel.norm();
    sensorData_.groundSpeed = vel.head<2>().norm();
    sensorData_.autopilotActive = true;
    sensorData_.hasGPSFix = true;
    sensorData_.timestamp = now;
    sensorData_.sequenceNumber++;

    // Publish
    onSensorData_(sensorData_);
}

