/*
 * SensingActuationIO.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: mircot
 */
#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>

#include "uavAP/Core/DataHandling/Content.hpp"

bool
SensingActuationIO::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSet<IPC, ITimeProvider>())
            {
                CPSLOG_ERROR << "SensingActuationIO missing dependencies.";
                return true;
            }
            if (!isSet<DataHandling>())
            {
                CPSLOG_DEBUG << "SensingActuationIO: DataHandling not set. Debugging disabled.";
            }
            auto ipc = get<IPC>();

            actuationPublisher_ = ipc->publish<ControllerOutput>("actuation");
            advancedControlPublisher_ = ipc->publish<AdvancedControl>("advanced_control");

            break;
        }
    case RunStage::NORMAL:
        {
            auto ipc = get<IPC>();
            sensorSubscription_ = ipc->subscribe<SensorData>("sensor_data", [this](const auto& sd)
            {
                onSensorData(sd);
            });
            powerSubscription_ = ipc->subscribe<PowerData>("power_data", [this](const auto& pd)
            {
                powerData_ = pd;
            });
            servoSubscription_ = ipc->subscribe<ServoData>("servo_data", [this](const auto& sd)
            {
                servoData_ = sd;
            });
            if (!sensorSubscription_.connected())
            {
                CPSLOG_ERROR << "SensorData in shared memory missing. Cannot continue.";
                return true;
            }
            if (auto dh = get<DataHandling>())
            {
                dh->addStatusFunction<SensorData>(
                    std::bind(&SensingActuationIO::getSensorData, this), Content::SENSOR_DATA);
                dh->addStatusFunction<PowerData>(
                    [this] { return powerData_; }, Content::POWER_DATA);
                dh->addStatusFunction<ServoData>(
                    [this] { return servoData_; }, Content::SERVO_DATA);
                dh->addStatusFunction<TimedValue<ControllerOutput>>(
                    [this]
                    {
                        std::unique_lock<SharedMutex> lock(controllerOutputMutex_);
                        return controllerOutput_;
                    }, Content::CONTROLLER_OUTPUT);

                dh->subscribeOnData<AdvancedControl>(Content::ADVANCED_CONTROL,
                                                     std::bind(&SensingActuationIO::onAdvancedControl, this,
                                                               std::placeholders::_1));
            }
            break;
        }
    default:
        {
            break;
        }
    }

    return false;
}

void
SensingActuationIO::setControllerOutput(const ControllerOutput& out)
{
    auto tp = get<ITimeProvider>();
    auto now = tp->now();
    std::unique_lock<SharedMutex> lock(controllerOutputMutex_);
    controllerOutput_ = {now, out};
    lock.unlock();
    actuationPublisher_.publish(out);
}

void
SensingActuationIO::onSensorData(const SensorData& data)
{
    std::unique_lock<SharedMutex> lock(mutex_);
    sensorData_ = data;
    lock.unlock();
    onSensorData_(data);
}

boost::signals2::connection
SensingActuationIO::subscribeOnSensorData(const OnSensorData::slot_type& slot)
{
    return onSensorData_.connect(slot);
}

SensorData
SensingActuationIO::getSensorData() const
{
    std::unique_lock<SharedMutex> lock(mutex_);
    return sensorData_;
}

void
SensingActuationIO::onAdvancedControl(const AdvancedControl& control)
{
    CPSLOG_TRACE << "Camber Control: " << EnumMap<CamberControl>::convert(control.camberSelection);
    CPSLOG_TRACE << "Special Control: " << EnumMap<SpecialControl>::convert(control.specialSelection);
    CPSLOG_TRACE << "Throw Control: " << EnumMap<ThrowsControl>::convert(control.throwsSelection);
    CPSLOG_TRACE << "Camber Value: " << control.camberValue;
    CPSLOG_TRACE << "Special Value: " << control.specialValue;
    advancedControlPublisher_.publish(control);
}
